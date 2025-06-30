#!/usr/bin/env python3
import sys
import os
import argparse

# make sure `models/` is on PYTHONPATH
ROOT = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, ROOT)  
os.environ["LD_PRELOAD"] = "/usr/lib/aarch64-linux-gnu/libgomp.so.1"
# export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from message_filters import Subscriber, ApproximateTimeSynchronizer

import cv2
import numpy as np
import torch
from ultralytics import YOLO
from rotation_conversions import rotation_6d_to_matrix
from PointTransformer2 import PointTransformerModel
import math

class RealTimeInferenceEngine:
    """
    Real-time inference engine combining YOLOv8 instance segmentation
    and a 3D pose estimation PointTransformer model.
    """
    def __init__(
        self,
        seg_weights: str,
        pose_checkpoint: str,
        num_points: int = 1024,
        num_predictions: int = 100,
        rotation: str = "6D",
        device: str = None,
        use_half: bool = True,
    ):
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.use_half = use_half and (self.device != "cpu")

        # Load & optimize segmentation model
        self.seg_model = YOLO(seg_weights)
        # self.seg_model.model.fuse().to(self.device)
        # if self.use_half:
        #     self.seg_model.model.half()

        # Load & prepare 3D pose model
        self.pose_model = (
            PointTransformerModel
            .load_from_checkpoint(
                pose_checkpoint,
                rotation=rotation,
                rgb=False,
                num_predictions=num_predictions
            )
            .eval()
            .to(self.device)
        )
        if self.use_half:
            self.pose_model.half()

        self.num_points = num_points
        self.rotation = rotation

    @staticmethod
    def build_masked_pc_with_rgb(rgb: np.ndarray, depth: np.ndarray, mask: np.ndarray) -> torch.Tensor:
        H, W = depth.shape
        z = depth.flatten()
        m = mask.flatten()
        valid = (z > 0) & (m == 1)
        if not valid.any():
            return torch.zeros((0, 6), dtype=torch.float32)

        xs = np.tile(np.arange(W), H)[valid]
        ys = np.repeat(np.arange(H), W)[valid]
        zs = z[valid]
        flat = rgb.reshape(-1, 3)
        rs, gs, bs = flat[valid].T

        pts = np.stack([
            xs / W, ys / H, zs / H,
            rs / 255.0, gs / 255.0, bs / 255.0
        ], axis=1)
        return torch.from_numpy(pts).float()

    @staticmethod
    def downsample_or_pad(pc: torch.Tensor, num_points: int) -> torch.Tensor:
        n, d = pc.shape
        if n > num_points:
            idx = torch.randperm(n, device=pc.device)[:num_points]
            return pc[idx]
        elif n < num_points:
            pad = torch.zeros((num_points - n, d), device=pc.device, dtype=pc.dtype)
            return torch.cat([pc, pad], dim=0)
        return pc

    @staticmethod
    def compute_box(mask: np.ndarray, depth: np.ndarray) -> tuple:
        ys, xs = np.where(mask)
        if len(xs) == 0:
            return 0.0, 0.0, 0.0
        w_px = float(xs.max() - xs.min())
        h_px = float(ys.max() - ys.min())
        d_vals = depth[ys, xs]
        return w_px, h_px, float(d_vals.max() - d_vals.min())

    def infer(self, rgb_image: np.ndarray, depth_map: np.ndarray) -> dict:
        H, W = rgb_image.shape[:2]

        # 1) Instance segmentation
        results = self.seg_model.predict(
            source=rgb_image,
            task="segment",
            device=self.device,
            verbose=False,
            save=True,
        )
        det = results[0]
        raw_masks = det.masks.data if det.masks is not None else []
        bool_masks = []
        for m in raw_masks:
            m_np = m.cpu().numpy()
            m_resized = cv2.resize(m_np, (W, H), interpolation=cv2.INTER_NEAREST)
            bool_masks.append(m_resized > 0.5)

        # return early if no masks
        if not bool_masks:
            return {"masks": [], "poses": []}

        # 2) Point clouds for poses
        pcs = []
        for mask in bool_masks:
            pc6 = self.build_masked_pc_with_rgb(rgb_image, depth_map, mask)
            pc6 = self.downsample_or_pad(pc6, self.num_points)
            pcs.append(pc6[:, :3])
        batch_pc = torch.stack(pcs, dim=0).to(self.device)
        if self.use_half:
            batch_pc = batch_pc.half()

        # 3) Pose inference
        with torch.no_grad():
            preds = self.pose_model(batch_pc)
            pred6d = preds[:, 0, :]
            rot_mats = (
                rotation_6d_to_matrix(pred6d)
                if self.rotation.lower() == "6d"
                else pred6d
            )

        # 4) Assemble
        poses = []
        batch_pc_cpu = batch_pc.cpu().numpy()
        rot_cpu = rot_mats.cpu().numpy()
        for i, mask in enumerate(bool_masks):
            w_px, h_px, d_raw = self.compute_box(mask, depth_map)
            dims_norm = (w_px / W, h_px / H, d_raw / H)
            center = batch_pc_cpu[i].mean(axis=0).tolist()
            poses.append({
                "rotation_matrix": rot_cpu[i].tolist(),
                "center": center,
                "box_dims_norm": dims_norm
            })

        return {"masks": bool_masks, "poses": poses}


class RealTimeInferenceNode(Node):
    def __init__(self, seg_only: bool):
        super().__init__('real_time_inference_node')
        self.seg_only = seg_only
        self.bridge = CvBridge()
        self.engine = RealTimeInferenceEngine(
            seg_weights='best_v3.pt',
            pose_checkpoint='epoch=29-val_loss=0.1542.ckpt',
            num_points=1024
        )

        # Subscribers
        color_sub = Subscriber(self, Image, '/camera/color/image_rect_raw')
        depth_sub = Subscriber(self, Image, '/camera/depth/image_rect_raw')
        self.ts = ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=10,
            slop=0.05
        )
        self.ts.registerCallback(self.image_callback)
        self.get_logger().info(
            f'RealTimeInferenceNode started (seg_only={self.seg_only})'
        )

    def image_callback(self, color_msg: Image, depth_msg: Image):
        try:
            cv_color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            rgb = cv2.cvtColor(cv_color, cv2.COLOR_BGR2RGB)
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_map = depth.astype(np.float32)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # Run inference
        results = self.engine.infer(rgb, depth_map)
        num = len(results['masks'])

        if self.seg_only:
            self.get_logger().info(f'[Seg only] Found {num} instance(s).')
            return

        # Otherwise print full pose info
        self.get_logger().info(f'Found {num} instance(s).')
        for i, pose in enumerate(results['poses']):
            self.get_logger().info(
                #f'Instance {i}: center={pose["center"]}, '
                f'Rotation Norm={[math.degrees(x) for x in pose["box_dims_norm"]]} degrees, '
            )


def main():
    # parse segmentation-only flag
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--seg-only',
        action='store_true',
        help='Only run segmentation and print instance count'
    )
    args = parser.parse_args()

    rclpy.init()
    node = RealTimeInferenceNode(seg_only=args.seg_only)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
