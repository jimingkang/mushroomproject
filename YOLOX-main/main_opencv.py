import argparse
import cv2
import numpy as np
import torch


class yolox():
    def __init__(self, model, p6=False, confThreshold=0.5, nmsThreshold=0.5, objThreshold=0.5):
        with open('coco.names', 'rt') as f:
            self.class_names = f.read().rstrip('\n').split('\n')
        self.net = cv2.dnn.readNet(model)
        self.input_size = (640, 640)
        self.mean = (0.485, 0.456, 0.406)
        self.std = (0.229, 0.224, 0.225)
        if not p6:
            self.strides = [8, 16, 32]
        else:
            self.strides = [8, 16, 32, 64]
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.objThreshold = objThreshold
        #self.is_half = is_half  # 是否开启半精度
        #self.device = device  # 计算设备
        self.model = model  # Yolov5模型
        #self.img_torch = img_torch  # 图像缓冲区
    def preprocess(self, image):
        if len(image.shape) == 3:
            padded_img = np.ones((self.input_size[0], self.input_size[1], 3)) * 114.0
        else:
            padded_img = np.ones(self.input_size) * 114.0
        img = np.array(image)
        r = min(self.input_size[0] / img.shape[0], self.input_size[1] / img.shape[1])
        resized_img = cv2.resize(
            img, (int(img.shape[1] * r), int(img.shape[0] * r)), interpolation=cv2.INTER_LINEAR
        ).astype(np.float32)
        padded_img[: int(img.shape[0] * r), : int(img.shape[1] * r)] = resized_img
        image = padded_img

        image = image.astype(np.float32)
        image = image[:, :, ::-1]
        image /= 255.0
        image -= self.mean
        image /= self.std
        return image, r
    def demo_postprocess(self, outputs):
        grids = []
        expanded_strides = []
        hsizes = [self.input_size[0] // stride for stride in self.strides]
        wsizes = [self.input_size[1] // stride for stride in self.strides]

        for hsize, wsize, stride in zip(hsizes, wsizes, self.strides):
            xv, yv = np.meshgrid(np.arange(hsize), np.arange(wsize))
            grid = np.stack((xv, yv), 2).reshape(1, -1, 2)
            grids.append(grid)
            shape = grid.shape[:2]
            expanded_strides.append(np.full((*shape, 1), stride))

        grids = np.concatenate(grids, 1)
        expanded_strides = np.concatenate(expanded_strides, 1)
        outputs[..., :2] = (outputs[..., :2] + grids) * expanded_strides
        outputs[..., 2:4] = np.exp(outputs[..., 2:4]) * expanded_strides
        return outputs
    def nms(self, boxes, scores):
        """Single class NMS implemented in Numpy."""
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            inter = w * h
            ovr = inter / (areas[i] + areas[order[1:]] - inter)

            inds = np.where(ovr <= self.nmsThreshold)[0]
            order = order[inds + 1]

        return keep
    def multiclass_nms(self, boxes, scores):
        """Multiclass NMS implemented in Numpy"""
        final_dets = []
        num_classes = scores.shape[1]
        for cls_ind in range(num_classes):
            cls_scores = scores[:, cls_ind]
            valid_score_mask = cls_scores > self.confThreshold
            if valid_score_mask.sum() == 0:
                continue
            else:
                valid_scores = cls_scores[valid_score_mask]
                valid_boxes = boxes[valid_score_mask]
                keep = self.nms(valid_boxes, valid_scores)
                if len(keep) > 0:
                    cls_inds = np.ones((len(keep), 1)) * cls_ind
                    dets = np.concatenate([valid_boxes[keep], valid_scores[keep, None], cls_inds], 1)
                    final_dets.append(dets)
        if len(final_dets) == 0:
            return None
        return np.concatenate(final_dets, 0)
    def vis(self, img, boxes, scores, cls_ids):
        for i in range(len(boxes)):
            box = boxes[i]
            cls_id = int(cls_ids[i])
            score = scores[i]
            if score < self.confThreshold:
                continue
            x0 = int(box[0])
            y0 = int(box[1])
            x1 = int(box[2])
            y1 = int(box[3])

            text = '{}:{:.1f}%'.format(self.class_names[cls_id], score * 100)
            font = cv2.FONT_HERSHEY_SIMPLEX
            txt_size = cv2.getTextSize(text, font, 0.4, 1)[0]
            cv2.rectangle(img, (x0, y0), (x1, y1), (0, 0, 255), 2)
            cv2.rectangle(img, (x0, y0 + 1), (x0 + txt_size[0] + 1, y0 + int(1.5 * txt_size[1])), (255, 255, 255), -1)
            cv2.putText(img, text, (x0, y0 + txt_size[1]), font, 0.4, (0, 255, 0), thickness=1)
        return img
    def detect(self, srcimg):
        img, ratio = self.preprocess(srcimg)
        blob = cv2.dnn.blobFromImage(img)
        self.net.setInput(blob)
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        predictions = self.demo_postprocess(outs[0])[0]

        boxes = predictions[:, :4]
        scores = predictions[:, 4:5] * predictions[:, 5:]

        boxes_xyxy = np.ones_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2.
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2.
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2.
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2.
        boxes_xyxy /= ratio
        dets = self.multiclass_nms(boxes_xyxy, scores)
        if dets is not None:
            final_boxes, final_scores, final_cls_inds = dets[:, :4], dets[:, 4], dets[:, 5]
            srcimg = self.vis(srcimg, final_boxes, final_scores, final_cls_inds)
        return srcimg


if __name__ == '__main__':
    parser = argparse.ArgumentParser("opencv inference sample")
    parser.add_argument("--model", type=str, default="yolox_x_mushroom.onnx", help="Input your onnx model.")
    parser.add_argument("--image_path", type=str, default='mushroom.jpg', help="Path to your input image.")
    parser.add_argument("--score_thr", type=float, default=0.3, help="Score threshould to filter the result.")
    parser.add_argument("--with_p6", action="store_true", help="Whether your model uses p6 in FPN/PAN.")
    args = parser.parse_args()
    net = yolox(args.model, p6=args.with_p6, confThreshold=args.score_thr)
    srcimg = cv2.imread(args.image_path)
    srcimg = net.detect(srcimg)

    winName = 'Deep learning object detection in OpenCV'
    cv2.namedWindow(winName, cv2.WINDOW_NORMAL)
    cv2.imshow(winName, srcimg)
    cv2.waitKey(0)
    cv2.destroyAllWindows()