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
from deep_sort_realtime.deepsort_tracker import DeepSort

from ultralytics.utils import LOGGER
LOGGER.setLevel("ERROR")
import numpy as np 

def transform_point_in_T3_to_T0(p3, theta1, theta2, theta3, l1=0.325, l2=0.275, l3=0.225):
    theta1, theta2, theta3=np.deg2rad(theta1), np.deg2rad(theta2), np.deg2rad(theta3)

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
        #self.reset_deep_sort_srv = self.create_service(Trigger, 'reset_deep_sort', self.reset_service)
        self.tracker = DeepSort(max_iou_distance=0.9, max_age=10,n_init=3, nms_max_overlap=0.75,embedder="torchreid")


        self.mean_height_publisher=self.create_publisher(Float32, "mean_height",10)
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.publisher_color_image = self.create_publisher(Image, 'color_image', 10)
        self.publisher_depth_image = self.create_publisher(Image, 'depth_image', 10)
        self.publisher_mushroom_location = self.create_publisher(Point, 'mushroom_location', 10)
        self.publisher_color_image_detection = self.create_publisher(Image, 'color_image_detection', 10)
        self.bridge = CvBridge()
        self.model = YOLO("/home/cotrobot/gripper/yolo11n_seg_640_abu 2.pt")
        self.results=[]
        self.tracked_point_T0 = None     # previously tracked mushroom global point
        self.missed_frames = 0           # how many frames the tracked one was missing
        self.switch_threshold = 5        # number of frames before switching
        self.distance_threshold = 0.05 
        #self.reset()
    
    def update_robot_angles(self,msg):
        self.theta1=msg.angle1
        self.theta2=msg.angle2
        self.theta3=msg.angle3



    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        self.color_intrin = color_frame.profile.as_video_stream_profile().intrinsics

        if not depth_frame or not color_frame:
            return

        self.color_image = np.asanyarray(color_frame.get_data())
        self.depth_image = np.asanyarray(depth_frame.get_data())

        color_msg = self.bridge.cv2_to_imgmsg(self.color_image, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_image, encoding="16UC1")

        self.publisher_color_image.publish(color_msg)
        self.publisher_depth_image.publish(depth_msg)
        msg = Float32()
        msg.data = np.mean(self.depth_image)
        self.mean_height_publisher.publish(msg)
        self.mean_height_publisher.publish(msg)

        img = self.color_image.copy()

        results = self.model(self.color_image)
        self.detections = results[0].boxes
        #arm length 22.5
        point_T0=None
        id=0
        self.deep_sort()
        if self.results:
            for box,id in self.results:
                x1, y1, x2, y2 = map(int, box)
                cv2.putText(img, str(id),  (x1, y1 - 10),  cv2.FONT_HERSHEY_SIMPLEX,  0.5,(0, 255, 0),  2 )
                cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
                x,y=int((x1+x2)/2),int((y1+y2)/2)
                depth_value=np.mean(self.depth_image[y-15:y+15,x-15:x+15])*0.01
                point=rs.rs2_deproject_pixel_to_point(self.color_intrin, [x, y],depth_value)
                cv2.putText(img, f"{round(point[0],2)},{round(point[1],2)},{round(point[2],2)}",  (x1, y1 - 20),  cv2.FONT_HERSHEY_SIMPLEX,  0.5,(0, 255, 0),  2 )
        height, width, _ = img.shape
        center_x, center_y = width // 2, height // 2

        # Draw horizontal and vertical lines
        cv2.line(img, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)
        cv2.line(img, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)

        # Optional: add a small center dot
        cv2.circle(img, (center_x, center_y), 3, (0, 0, 255), -1)

        color_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.publisher_color_image_detection.publish(color_msg)
        #     x,y=int((x1+x2)/2),int((y1+y2)/2)
        #     point=rs.rs2_deproject_pixel_to_point(self.color_intrin, [x, y], np.mean(self.depth_image[y-5:y+5,x-5:x+5])*0.0001)
            
        #     point_T0=transform_point_in_T3_to_T0(point, self.theta1, self.theta2, self.theta3)[:2]
        #     #print("point camera",round(point[0],2),round(point[1],2),"Point global",round(point_T0[0],2),round(point_T0[1],2))
        #     if np.mean(self.depth_image[y1:y2,x1:x2])>np.mean(self.depth_image):
        #         continue
        #     conf = float(box.conf[0])
        #     cls = int(box.cls[0])
        #     cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        #     cv2.putText(img, f"ID: {id} conf {round(conf,2)}", (x1, y1 - 10),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        #     id+=1
        #     if np.linalg.norm(np.array(point_T0))<0.82:
        #         self.results.append([point,point_T0,[x1, y1, x2, y2]])
        # score=[]
        # if len(self.results) == 0:
        #     self.missed_frames += 1
        # else:
        #     # Convert results into a list of (point_T0, box)
        #     point_list = [r[1] for r in self.results]
        #     boxes = [r[2] for r in self.results]
            
        #     selected_index = None

        #     if self.tracked_point_T0 is not None:
        #         # Find closest detection to the previously tracked one
        #         distances = [np.linalg.norm(np.array(p) - np.array(self.tracked_point_T0)) for p in point_list]
        #         min_dist = min(distances)
        #         min_idx = distances.index(min_dist)

        #         # If close enough, keep tracking it
        #         if min_dist < self.distance_threshold:
        #             selected_index = min_idx
        #             self.missed_frames = 0
        #         else:
        #             # Tracked one lost
        #             self.missed_frames += 1

        #     # If no current tracked target or we've missed too many frames, switch
        #     if selected_index is None or self.missed_frames > self.switch_threshold:
        #         self.missed_frames = 0
        #         # Pick new mushroom based on score (your original logic)
        #         score = []
        #         for point1, _, _ in self.results:
        #             total_distance = 0
        #             for point2, _, _ in self.results:
        #                 total_distance += np.linalg.norm(np.array(point1[:2]) - np.array(point2[:2])) / 2
        #             total_distance -= np.linalg.norm(point1[:2]) / 3
        #             total_distance += point1[-1] / 10
        #             score.append(total_distance)
        #         selected_index = score.index(max(score))
            
        #     # Update the tracked mushroom
        #     track = self.results[selected_index]
        #     point, point_T0, [x1, y1, x2, y2] = track
        #     self.tracked_point_T0 = point_T0

        #     # Draw highlight box
        #     cv2.rectangle(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        # if point_T0 is not None:
        #     publish_point=Point()
        #     publish_point.x=point_T0[0]
        #     publish_point.y=-point_T0[1]
        #     self.publisher_mushroom_location.publish(publish_point)
        # color_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        # self.publisher_color_image_detection.publish(color_msg)
        

        # self.get_logger().info('Publishing images')
    
    def deep_sort(self):
        if self.detections is not None:
            formatted_detections = []
            for box in self.detections:
                
                xyxy = box.xyxy[0].cpu().numpy()
                conf = box.conf.item()

                left = int(xyxy[0])
                top = int(xyxy[1])
                right = int(xyxy[2])
                bottom = int(xyxy[3])
                if (left-right)*(top-bottom)<1000 or float(box.conf[0])<0.8:
                    continue
                conf = float(box.conf[0])
                cls = int(box.cls[0])

                width = right - left
                height = bottom - top

                formatted_detections.append([[left, top, width, height], conf, 0])  
            tracks = self.tracker.update_tracks(formatted_detections, frame=self.color_image)  
            self.results = [[list(track.to_ltrb()), int(track.track_id)] for track in tracks]
            
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
