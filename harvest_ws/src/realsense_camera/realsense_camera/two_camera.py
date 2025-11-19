import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
from example_interfaces.srv import Trigger
from example_interfaces.msg import Float32,Float32MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


from ultralytics.utils import LOGGER
LOGGER.setLevel("ERROR")
import numpy as np 

#jimmy add 
import redis
import ros2_numpy as rnp
import numpy as np
import cv2
from numpy import empty
from loguru import logger
from .camera_ipcam import Predictor

import pyrealsense2 as rs
from numpy import empty
import torch
import torch.backends.cudnn as cudnn
from yolox.data.data_augment import ValTransform
from yolox.data.datasets import COCO_CLASSES
from yolox.exp import get_exp
from yolox.utils import fuse_model, get_model_info, postprocess, setup_logger, vis

from roboflow import Roboflow
import supervision as sv
import cv2

from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,PointCloud2,PointField
from rclpy.qos import qos_profile_sensor_data
from bboxes_ex_msgs.msg import BoundingBoxes,BoundingBoxesCords
from bboxes_ex_msgs.msg import BoundingBox,BoundingBoxCord
#from yolox_ros_py_utils.utils import yolox_py
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String,Int32
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path

broker="172.23.66.117"
redis_server='172.23.248.33'

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)

def transform_point_in_T3_to_T0(p3, theta1, theta2, theta3, l1=0.325, l2=0.275, l3=0.225):
    #theta1, theta2, theta3=np.deg2rad(theta1), np.deg2rad(theta2), np.deg2rad(theta3)

    T01 = np.array([
        [np.cos(theta1), -np.sin(theta1), 0, l1 * np.cos(theta1)],
        [np.sin(theta1),  np.cos(theta1), 0, l1 * np.sin(theta1)],
        [0,               0,              1, 0],
        [0,               0,              0, 1]
    ])
    T12 = np.array([
        [np.cos(theta2), -np.sin(theta2), 0, l2 * np.cos(theta2)],
        [np.sin(theta2),  np.cos(theta2), 0, l2 * np.sin(theta2)],
        [0,               0,              1, 0],
        [0,               0,              0, 1]
    ])
    T23 = np.array([
        [np.cos(theta3), -np.sin(theta3), 0, l3 * np.cos(theta3)],
        [np.sin(theta3),  np.cos(theta3), 0, l3 * np.sin(theta3)],
        [0,               0,              1, 0],
        [0,               0,              0, 1]
    ])

    T03 = T01 @ T12 @ T23

    # Convert point to homogeneous coordinates
    p3_hom = np.array([p3[0], p3[1], p3[2], 1])

    # Transform point to T0
    p0_hom = T03 @ p3_hom

    # Return only x, y, z
    return p0_hom[:3]

class Camera2(Node):

    def __init__(self):
        super().__init__('dual_realsense_node')
        
        # Realsense pipeline setup
        #self.pipeline = rs.pipeline()
        #self.config = rs.config()
        #self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        #self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        #self.pipeline.start(self.config)
        #self.align = rs.align(rs.stream.color)
        
        self.theta1,self.theta2,self.theta3=0,0,0
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.mean_height_publisher=self.create_publisher(Float32, "mean_height",10)
        self.timer_period = 0.1  # seconds
        #self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.publisher_color_image = self.create_publisher(Image, 'color_image', 10)
        self.publisher_depth_image = self.create_publisher(Image, 'depth_image', 10)
        self.publisher_mushroom_location = self.create_publisher(Point, 'mushroom_location', 10)
        self.publisher_color_image_detection = self.create_publisher(Image, 'color_image_detection', 10)
        self.bridge = CvBridge()
        self.model = YOLO("/home/cotrobot/gripper/yolo11n_seg_640_abu_2.pt")#YOLO("/home/cotrobot/mushroomproject/train13_chengcheng_7917.pt")
        self.results=[]
        self.tracked_point_T0 = None     
        self.missed_frames = 0         
        self.switch_threshold = 5        
        self.distance_threshold = 0.05 
        self.pc2_publisher = self.create_publisher(PointCloud2, "pointcloud", 500)
        self.decimation = rs.decimation_filter(2)
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole = rs.hole_filling_filter()
        self.joint_state_suscriber = self.create_subscription(JointState,"/joint_states",self.update_joint_states,10)
        #self.reset()

        self.bridge = CvBridge()
        self.d435_intrinsics = None
        self.d405_intrinsics = None
        #serials = ['027422070780','128422272136']
        serials = ['405622072832','128422270555']
        
        #serials = ['128422272136']# 128422272400
        self.pipelines = []
        self.aligns = []   # 每个相机独立一个 align 对象

        # 初始化 pipeline 和 align
        for s in serials:
            pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(s)
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
            pipeline.start(cfg)
            profile = pipeline.get_active_profile()
            # Get color stream intrinsics
            color_stream = profile.get_stream(rs.stream.color)  # rs.video_stream_profile
            cameraInfo = color_stream.as_video_stream_profile().get_intrinsics()
            if s == '405622072832':

                try:
                    if self.d435_intrinsics:
                        return
                    self.d435_intrinsics = rs.intrinsics()
                    self.d435_intrinsics.width = cameraInfo.width
                    self.d435_intrinsics.height = cameraInfo.height
                    self.d435_intrinsics.ppx = cameraInfo.ppx
                    self.d435_intrinsics.ppy = cameraInfo.ppy
                    self.d435_intrinsics.fx = cameraInfo.fx
                    self.d435_intrinsics.fy = cameraInfo.fy
                    self.d435_intrinsics.model = cameraInfo.model
                    self.d435_intrinsics.coeffs = cameraInfo.coeffs
                except CvBridgeError as e:
                    self.get_logger().info(e)
                    return
            else:
                try:
                    if self.d405_intrinsics:
                        return
                    self.d405_intrinsics = rs.intrinsics()
                    self.d405_intrinsics.width = cameraInfo.width
                    self.d405_intrinsics.height = cameraInfo.height
                    self.d405_intrinsics.ppx = cameraInfo.ppx
                    self.d405_intrinsics.ppy = cameraInfo.ppy
                    self.d405_intrinsics.fx = cameraInfo.fx
                    self.d405_intrinsics.fy = cameraInfo.fy
                    self.d405_intrinsics.model = cameraInfo.model
                    self.d405_intrinsics.coeffs = cameraInfo.coeffs
                except CvBridgeError as e:
                    self.get_logger().info(e)
                    return   
            # align depth 到 color
            align_to = rs.stream.color
            align = rs.align(align_to)
            self.pipelines.append(pipeline)
            self.aligns.append(align)

        self.pub1 = self.create_publisher(Image, '/d435/color/image_raw', 10)
        self.pub2 = self.create_publisher(Image, '/d405/color/image_raw', 10)

        self.d435_pub_bounding_boxes = self.create_publisher(String,"/d435/yolox/bounding_boxes", 1)
        self.d435_pub_boxes_img = self.create_publisher(Image,"/d435/yolox/boxes_image", 10)
        #self.d435_sub_info = self.create_subscription(CameraInfo, self.d435_depth_info_topic, self.D435_imageDepthInfoCallback, qos)

        self.d405_adj_pub_bounding_boxes = self.create_publisher(String,"/d405/yolox/adj_bounding_boxes", 1)
        self.d405_pub_boxes_img = self.create_publisher(Image,"/d405/yolox/boxes_image", 10)



        self.timer = self.create_timer(0.03, self.newyolo11_timer_callback)

        #self.setting_yolox_exp()
    

    def update_joint_states(self,msg):
        self.theta1,self.theta2,self.theta3=msg.position[1],msg.position[2],msg.position[3]
    
    def capture_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        # Apply filters
        depth_frame = self.spatial.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        depth_frame = self.hole.process(depth_frame)
            
        if not depth_frame or not color_frame:
            return None, None

        self.color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        pts=[]
        for x in range(0,480,10):
            for y in range(0,640,10):
                depth_value = np.mean(depth_image[x,y]) * 0.0001

                point = rs.rs2_deproject_pixel_to_point(self.color_intrin, [y, x], depth_value)
                if np.linalg.norm(np.array([point[0],point[1],-point[2]]))>1:
                    continue
                pts.append([point[0],-point[1],-point[2]])
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "gripper_base"

        cloud_msg = pc2.create_cloud_xyz32(header, pts)

        self.pc2_publisher.publish(cloud_msg)
        self.get_logger().info("Published point cloud")
        return color_image, depth_image


    def publish_raw_images(self, color_image, depth_image):
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        self.publisher_color_image.publish(color_msg)
        self.publisher_depth_image.publish(depth_msg)

        msg = Float32()
        msg.data = np.mean(depth_image)
        self.mean_height_publisher.publish(msg)


    def detect_objects(self, color_image, depth_image):
        results = self.model(color_image)
        detections = results[0].boxes
        valid_results = []

        for box in detections:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])

            # Filter small or low-confidence detections
            if abs(x1 - x2) * abs(y1 - y2) < 800 or conf < 0.8:
                continue

            # Compute central depth
            x, y = int((x1 + x2) / 2), int((y1 + y2) / 2)
            depth_value = np.mean(depth_image[y,x]) * 0.0001

            point = rs.rs2_deproject_pixel_to_point(self.color_intrin, [x, y], depth_value)
            point=[point[0],-point[1],-point[2]]
            point_T0 =[i for i in transform_point_in_T3_to_T0(point, self.theta1, self.theta2, self.theta3)[:2]]
            point_T0.append(depth_value)
            point_T0=np.array(point_T0)
            if np.linalg.norm(point_T0[:2])>0.825:
                continue
            
            if np.mean(depth_image[y1:y2, x1:x2]) > np.mean(depth_image):
                continue

            p1=rs.rs2_deproject_pixel_to_point(self.color_intrin, [x1, y1], depth_value)
            
            p2=rs.rs2_deproject_pixel_to_point(self.color_intrin, [x2, y2], depth_value)
            
            if np.linalg.norm(np.array(point_T0[:2])) < 0.82:
                valid_results.append([point, point_T0,[x1,y1,x2,y2],[abs(p2[0]-p1[0]),abs(p2[1]-p1[1]),-abs(depth_value)]])

        return valid_results


    def select_target(self, results):
        if len(results) == 0:
            self.missed_frames += 1
            return None

        point_list = [r[1] for r in results]
        boxes = [r[2] for r in results]
        selected_index = None

        # Continue tracking existing target if close enough
        if self.tracked_point_T0 is not None:
            distances = [np.linalg.norm(np.array(p[:2]) - np.array(self.tracked_point_T0)) for p in point_list]
            min_dist = min(distances)
            min_idx = distances.index(min_dist)
            if min_dist < self.distance_threshold:
                selected_index = min_idx
                self.missed_frames = 0
            else:
                self.missed_frames += 1

        # Pick new target if tracking lost
        if selected_index is None or self.missed_frames > self.switch_threshold:
            self.missed_frames = 0
            scores = []
            for point1, _, _ ,_ in results:
                total_distance = sum(
                    np.linalg.norm(np.array(point1[:2]) - np.array(p2[:2])) / 2
                    for p2, _, _,_ in results
                )
                total_distance -= np.linalg.norm(point1[:2]) / 3
                total_distance += point1[-1] / 10
                scores.append(total_distance)
            selected_index = scores.index(max(scores))

        return results[selected_index]


    def publish_marker(self, point_T0,scale):
        
        marker = Marker()
        marker.header.frame_id = "gripper_base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mushroom"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = point_T0[0]
        marker.pose.position.y = point_T0[1]
        marker.pose.position.z = -point_T0[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01 if scale[0]<0.01 else scale[0]
        marker.scale.y = 0.01 if scale[1]<0.01 else scale[1]
        marker.scale.z = 0.01 if scale[2]<0.01 else scale[2]
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.publisher.publish(marker)


    def timer_callback(self):
        # Step 1: Capture
        color_image, depth_image = self.capture_frames()
        if color_image is None:
            return

        # Step 2: Publish base images
        self.publish_raw_images(color_image, depth_image)

        # Step 3: Detect
        results = self.detect_objects(color_image, depth_image)
        
        # Step 4: Select target
        selected = self.select_target(results)
        img = color_image.copy()
        for point, point_T0, bbox,scale in results:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0  , 0), 2)
        # Step 5: Draw and publish
        if selected:
            point, point_T0, bbox,scale = selected
            self.tracked_point_T0 = point[:2]
            self.publish_marker([point[0],point[1],point_T0[-1]], scale)

            x1, y1, x2, y2 = bbox
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        else:
            self.missed_frames += 1

        color_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher_color_image_detection.publish(color_msg)

        

        # self.get_logger().info('Publishing images')
    
    # def deep_sort(self):
    #     if self.detections is not None:
    #         formatted_detections = []
    #         for box in self.detections:
                
    #             xyxy = box.xyxy[0].cpu().numpy()
    #             conf = box.conf.item()

    #             left = int(xyxy[0])
    #             top = int(xyxy[1])
    #             right = int(xyxy[2])
    #             bottom = int(xyxy[3])
    #             if (left-right)*(top-bottom)<600 or float(box.conf[0])<0.7:
    #                 continue
    #             conf = float(box.conf[0])
    #             cls = int(box.cls[0])

    #             width = right - left
    #             height = bottom - top

    #             formatted_detections.append([[left, top, width, height], conf, 0])  
    #         tracks = self.tracker.update_tracks(formatted_detections, frame=self.color_image)  
    #         self.results = [[list(track.to_ltrb()), int(track.track_id)] for track in tracks]
            
    # def reset_service(self, request, response):
    #     self.reset()
    #     response.success = True
    #     response.message = "Deepsort reset successfully!"
    # def reset(self):
    #     self.tracker = DeepSort(max_iou_distance=1.5, max_age=5, nms_max_overlap=0.75)
    #     self.result=[]


        #jimmy add  
    def setting_yolox_exp(self) -> None:
        WEIGHTS_PATH = '../../weights/yolox_nano.pth'  #for no trt
        self.declare_parameter('imshow_isshow',True)
        self.declare_parameter('yolox_exp_py', '/home/jimmy/Downloads/mushroomproject/ros2_ws/src/YOLOX-ROS/yolox_ros_py/exps/yolox_nano.py')
        #self.declare_parameter('yolox_exp_py', 'yolox_vos_s.py')
        self.declare_parameter('fuse',False)
        self.declare_parameter('trt', True)
        self.declare_parameter('fp16', False)
        self.declare_parameter('legacy', False)
        self.declare_parameter('device', "gpu")
        # self.declare_parameter('', 0)
        self.declare_parameter('ckpt', WEIGHTS_PATH)
        self.declare_parameter('conf', 0.3)
        # nmsthre -> threshold
        self.declare_parameter('threshold', 0.55)
        # --tsize -> resize
        self.declare_parameter('resize', 640)
        self.declare_parameter('sensor_qos_mode', False)
        # =============================================================
        self.imshow_isshow = self.get_parameter('imshow_isshow').value

        exp_py = self.get_parameter('yolox_exp_py').value
        fuse = self.get_parameter('fuse').value
        trt = True#self.get_parameter('trt').value
        fp16 = self.get_parameter('fp16').value
        device = self.get_parameter('device').value
        ckpt = self.get_parameter('ckpt').value
        conf = self.get_parameter('conf').value
        legacy = self.get_parameter('legacy').value
        threshold = self.get_parameter('threshold').value
        
        input_shape_w = self.get_parameter('resize').value
        input_shape_h = input_shape_w

        self.sensor_qos_mode = self.get_parameter('sensor_qos_mode').value
        # ==============================================================
        # add for orbslam3
        #self.declare_parameter('--vocab_file', vocab_file)

        cudnn.benchmark = True
        exp = get_exp(exp_py, None)

        #BASE_PATH = os.getcwd()
        #file_name = os.path.join(BASE_PATH, "../YOLOX-main/YOLOX_outputs/yolox_voc_s/")
        file_name = "/home/jimmy/Downloads/mushroomproject/YOLOX-main/YOLOX_outputs/yolox_voc_s/"#ros2_ws/src/YOLOX-ROS/weights/tensorrt/"#os.path.join(BASE_PATH, "/src/YOLOX-ROS/weights/tensorrt/") #
        # os.makedirs(file_name, exist_ok=True)

        exp.test_conf = conf # test conf
        exp.threshold = threshold # nms threshold
        exp.test_size = (input_shape_h, input_shape_w) # test size

        model = exp.get_model()
        self.get_logger().info("Model Summary: {}".format(get_model_info(model, exp.test_size)))

        if device == "gpu":
            model.cuda()
            #if fp16:
            #    model.half() 
        # torch.cuda.set_device()
        # model.cuda()
        model.eval()

        # about not trt
        if not trt:
            if ckpt is None:
                ckpt_file = os.path.join(file_name, "best_ckpt.pth")
            else:
                ckpt_file = ckpt
            self.get_logger().info("loading checkpoint")
            ckpt = torch.load(ckpt_file, map_location="cpu")
            # load the model state dict
            model.load_state_dict(ckpt["model"],strict=False)
            self.get_logger().info("loaded checkpoint done.")

        # about fuse
        if fuse:
            self.get_logger().info("\tFusing model...")
            model = fuse_model(model)

        # TensorRT
        self.get_logger().info("trt: {},file_name:{}".format(trt,file_name))
        if trt:
            assert not fuse, "TensorRT model is not support model fusing!"
            trt_file = os.path.join(file_name, "model_trt.pth") 
            self.get_logger().info("trt_file:{}".format(trt_file))
            assert os.path.exists(
                trt_file
            ), "TensorRT model is not found!\n Run python3 tools/trt.py first!"
            model.head.decode_in_inference = False
            decoder = model.head.decode_outputs
            self.get_logger().info("Using TensorRT to inference")
        else:
            trt_file = None
            decoder = None
        self.predictor = Predictor(model, exp, COCO_CLASSES, trt_file, decoder, device, fp16, legacy)
        self.get_logger().info("Yolox setup complete.")
    def newyolo11_timer_callback(self):
        for j, (pipe, align) in enumerate(zip(self.pipelines, self.aligns)):
            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())
            results = self.model(color_img)
            detections = results[0].boxes
            valid_results = []
            bbox=String()
            for box in detections:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])

                # Filter small or low-confidence detections
                if abs(x1 - x2) * abs(y1 - y2) < 800 or conf < 0.8:
                    continue
                # Compute central depth
                x, y = int((x1 + x2) / 2), int((y1 + y2) / 2)
                depth_value = depth_frame.get_distance(x, y)
                X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().get_intrinsics(),[x, y],depth_value)
                if (j==0 and Y>0 and Z<1.0)or j==1: 
                    print(f"3D({X:.3f}, {Y:.3f}, {Z:.3f})  depth={depth_value:.3f} m")
                    cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(color_img, f"conf={conf:.2f},depth={depth_value:.3f}",(x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                    bbox.data=f'{X},{Y},{Z}'
                    break
            if j==0 :
                #print(f"{j==0} d435 image publish")
                self.d435_pub_boxes_img.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
                if (r.get("mode")=="camera_ready"):
                    if bbox.data=="":
                        #self.get_logger().info("no valid bbox from d435")
                        continue
                    self.d435_pub_bounding_boxes.publish(bbox)
                    self.get_logger().info(" mushroom xyz in side camera: {}".format(bbox.data))
                    #r.set("mode","camera_done")

            elif j==1 :
                #print(f"{j==0} d405 image publish")
                self.d405_pub_boxes_img.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
                if  (r.get("mode")=="adjust_ready"):
                    if bbox.data=="" :
                        self.get_logger().info("no valid bbox from d405")
                        continue
                    self.get_logger().info("mushroom xyz in top camera:{},".format(bbox.data))
                    X,Y,Z=[float(x) for x in bbox.data.split(",")]
                    self.d405_adj_pub_bounding_boxes.publish(bbox)
            bbox.data=""    




    def new_timer_callback(self):
        for j, (pipe, align) in enumerate(zip(self.pipelines, self.aligns)):
            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())

            color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='16UC1')
            #(self.pub1 if j == 0 else self.pub2).publish(color_msg)

            outputs, img_info =self.model(color)# self.predictor.inference(color_img)
            self.get_logger().info("mode={},mode==adjust_ready,{}".format(r.get("mode"),r.get("mode")=="adjust_ready"))
            bbox=String()
            #if  (outputs is not None) : #and r.get("mode")=="camera_ready":# and r.get("scan")=="start" :#
            if outputs is None or len(outputs) == 0 or outputs[0] is None:
                self.get_logger().info("No detections returned from predictor.")
                continue  # or continue
            else:
                output = outputs[0]
                if isinstance(output, torch.Tensor):
                    output = output.cpu().numpy()

                boxes = output[:, 0:4]
                scores = output[:, 4] * output[:, 5]
                class_ids = output[:, 6].astype(int)
                self.get_logger().info(f"Detected {len(boxes)} objects.")

                for i in range(len(boxes)):
                    x1, y1, x2, y2 = map(int, boxes[i])
                    conf = scores[i]
                    cls_id = class_ids[i]
                    print(f"Object {i}: class={cls_id}, conf={conf:.2f}, box=({x1},{y1},{x2},{y2})")
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    # 获取中心点深度
                    depth_value = depth_frame.get_distance(cx, cy)
                    X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().get_intrinsics(),[cx, cy],depth_value)
                    X2, Y2, Z2 = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().get_intrinsics(),[x2, y2],depth_frame.get_distance(x2, y2))
                    if (j==0 and Y>0 and Z<1.0 and abs(X2-X1)<0.1)or j==1: 
                        #print(f"3D({X:.3f}, {Y:.3f}, {Z:.3f})  depth={depth_value:.3f} m")
                        cv2.rectangle(color_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(color_img, f"id={cls_id}:conf={conf:.2f},depth={depth_value:.3f}",(x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 255, 0), 2)
                        bbox.data=f'{X},{Y},{Z}'
                        break
            if j==0 :
                #print(f"{j==0} d435 image publish")
                self.d435_pub_boxes_img.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
                if 1:#(r.get("mode")=="camera_ready"):
                    if bbox.data=="":
                        self.get_logger().info("no valid bbox from d435")
                        continue
                    self.d435_pub_bounding_boxes.publish(bbox)
                    self.get_logger().info(" mushroom xyz in side camera: {}".format(bbox.data))
                    #r.set("mode","camera_done")

            elif j==1 :
                print(f"{j==0} d405 image publish")
                self.d405_pub_boxes_img.publish(self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8'))
                if  (r.get("mode")=="adjust_ready"):
                    if bbox.data=="" :
                        self.get_logger().info("no valid bbox from d405")
                        continue
                    self.get_logger().info("mushroom xyz in top camera before rotation:{},".format(bbox.data))
                    X,Y,Z=[float(x) for x in bbox.data.split(",")]
                    self.d405_adj_pub_bounding_boxes.publish(bbox)
            bbox.data=""


def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera2()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

