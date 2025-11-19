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

class Camera(Node):

    def __init__(self):
        super().__init__('camera_node')
        
        # Realsense pipeline setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        self.pipeline.start(self.config)
        self.align = rs.align(rs.stream.color)
        
        self.theta1,self.theta2,self.theta3=0,0,0
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.mean_height_publisher=self.create_publisher(Float32, "mean_height",10)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.publisher_color_image = self.create_publisher(Image, 'color_image', 10)
        self.publisher_depth_image = self.create_publisher(Image, 'depth_image', 10)
        self.publisher_mushroom_location = self.create_publisher(Point, 'mushroom_location', 10)
        self.publisher_color_image_detection = self.create_publisher(Image, 'color_image_detection', 10)
        self.bridge = CvBridge()
        self.model = YOLO("/home/cotrobot/gripper/yolo11n_seg_640_abu 2.pt")
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


def main(args=None):
    rclpy.init(args=args)
    camera_node = Camera()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
