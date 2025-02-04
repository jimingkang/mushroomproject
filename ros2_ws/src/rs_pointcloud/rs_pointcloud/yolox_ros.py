#!/usr/bin/env python
import argparse
from importlib import import_module
import os

import redis
from flask import Flask, render_template, Response, jsonify
import signal
from threading import Event
import requests
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt
# from flask_socketio import SocketIO
import redis
import serial
import datetime
import time
import os
from time import sleep
import math
from flask_cors import CORS, cross_origin

from pathlib import Path
import time
#from yolox_ros_py.HitbotInterface import HitbotInterface
import redis


#import ros_numpy
#from ros_numpy import numpy_msg
import ros2_numpy as rnp

import numpy as np
import cv2
from numpy import empty
from loguru import logger
from yolox_ros_py.camera_ipcam import Predictor

import pyrealsense2 as pyrs
from numpy import empty

from std_msgs.msg import String  


import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,PointCloud2,PointField
from rclpy.qos import qos_profile_sensor_data
from bboxes_ex_msgs.msg import BoundingBoxes,BoundingBoxesCords
from bboxes_ex_msgs.msg import BoundingBox,BoundingBoxCord

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from scipy.spatial.transform import Rotation as R
# camera intrinsics
W, H = 848-12, 480+10
F = 430.79
K = np.array([[F,0,W//2],[0,F,H//2],[0,0,1]])
#mapp = Mapp(W, H)
frame=None




import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

i = 0



broker="172.27.34.62"
redis_server="172.27.34.62"


pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)




app = Flask(__name__)
app.config['DEBUG'] = False

cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'


y = 0





ros_class=None





def gen(camera):
    """Video streaming generator function."""
    yield b'--frame\r\n'
    while True:
        frame = camera.get_frame()
        yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'


#@app.route('/video_feed')
#def video_feed():
#    """Video streaming route. Put this in the src attribute of an img tag."""
#    return Response(gen(Camera()),
#                    mimetype='multipart/x-mixed-replace; boundary=--frame')



@app.route('/video_feed')
def video_feed():
    global ros_class
    return Response(ros_class.gen(Camera()),mimetype='multipart/x-mixed-replace; boundary=frame')







#bounding_boxes=None
bboxes_msg=None
result_img_rgb=None
img_rgb=None
points_xyz_rgb=np.asarray([[]])
pointsxyzrgb_total=np.asarray([[0,0,0,0,0,0]])
global_points_xyz_rgb_list=np.asarray([()])
i=0
class yolox_ros(Node):
    def __init__(self) -> None:
        raw_image_topic = '/camera/camera/color/image_raw'
        depth_image_topic = '/camera/camera/depth/image_rect_raw'
        depth_info_topic = '/camera/camera/depth/camera_info'
        move_x="/move/x"

        # ROS2 init
        super().__init__('yolox_ros')

        #self.setting_yolox_exp()


        self.bridge = CvBridge()
        self.i=0
        #self.pub = self.create_publisher(BoundingBoxes,"/yolox/bounding_boxes", 10)
        self.pub_bounding_boxes_cords = self.create_publisher(BoundingBoxesCords,"/yolox/bounding_boxes_cords", 1)
        self.pub_boxes_img = self.create_publisher(Image,"/yolox/boxes_image", 10)

        self.pub_pointclouds = self.create_publisher(PointCloud2,'/yolox/pointclouds', 10)
        self.sub_depth_image = self.create_subscription(Image, depth_image_topic, self.imageDepthCallback, 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.sub_move_xy_info = self.create_subscription(String, move_x, self.MoveXYZCallback, 1)
        self.sub_count_info = self.create_subscription(String, "/count", self.CountCallback, 1)
        self.pub_count2 = self.create_publisher(String,'/count2', 1)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        #if (self.sensor_qos_mode):
        #    self.sub = self.create_subscription(Image,raw_image_topic,self.imageflow_callback, qos_profile_sensor_data)
        #else:
        self.sub = self.create_subscription(Image,raw_image_topic,self.imageflow_callback, 10)
        self.count="0"
  
   
     


    def CountCallback(self,data):
        self.count=data
        logger.info("count{}".format(self.count))

    def imageflow_callback(self,msg:Image) -> None:
            global bboxes_msg,result_img_rgb,img_rgb,mapp,frame
            img_rgb = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            #logger.info("img_rgb : {},".format((img_rgb)))
            if img_rgb is not None:
                #outputs, img_info = self.predictor.inference(img_rgb)
                #logger.info("outputs : {},".format((outputs)))
                result_img_rgb=img_rgb
                #logger.info("result_img_rgb : {},".format((result_img_rgb)))

 #property uchar green
 #   property uchar blue
  #    property uchar i
    def create_point_cloud_file2(self,vertices, filename):
        ply_header = '''ply
    format ascii 1.0
    element vertex %(vert_num)d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
   
    end_header
    '''
        with open(filename, 'w') as f:
            f.write(ply_header %dict(vert_num=len(vertices)))
            np.savetxt(f,vertices,'%f %f %f %d %d %d')
    def pose_estimation(self,depth,raw_img,bounding_boxes):
        print(" pose_estimation bounding_boxes:", bounding_boxes)
        # Convert depth image to proper format if necessary
        if depth.dtype != np.float32:
            depth = depth.astype(np.float32)
        # Convert depth units if necessary (e.g., scaling)
        # depth = depth * depth_scale  # Uncomment and set depth_scale if needed
        # Extract annotations for the specific image
        #image_id = get_image_id_by_name(image_name, coco)
        #annotations = [ann for ann in coco['annotations'] if ann['image_id'] == image_id]

        # Generate the 3D point cloud for the entire image using your provided code
        (height, width, _) = raw_img.shape
        xx, yy = np.meshgrid(np.arange(0, width), np.arange(0, height))

        # Flatten the arrays to create a point cloud
        x = xx.flatten()
        y = yy.flatten()
        z = depth.flatten()

        # Remove invalid depth points
        valid = (z > 0) & (~np.isnan(z))
        x = x[valid]
        y = y[valid]
        z = z[valid]

        # Stack x, y, z coordinates into a point array
        points = np.vstack((x, y, z)).T

        # Reshape the RGB image for color mapping and normalize colors
        colors = raw_img.reshape(-1, 3)[valid]
        colors = (colors).astype(np.uint8)  # Ensure colors are uint8 for PyVista

        # Camera intrinsic parameters (adjust if known)
        fx = fy = 382  # Focal length in pixels
        cx = width / 2.0
        cy = height / 2.0

        # Convert pixel coordinates to 3D coordinates in camera space
        X = (x - cx) * z / fx
        Y = (y - cy) * z / fy
        Z = z

        # Stack into Nx3 array
        points_3d = np.vstack((X, Y, Z)).T

        # Create a PyVista point cloud
        #cloud = pv.PolyData(points_3d)
        #cloud["RGB"] = colors

        # Create a PyVista plotter
        #plotter = pv.Plotter()

        # Add the point cloud to the plotter and set up color mapping
        #plotter.add_points(cloud, scalars="RGB", rgb=True, point_size=1)

        # Set plot labels
        #plotter.set_background("white")
        #plotter.add_axes()

        # Prepare to store rotation values and normals
        rotation_results = []
    
        # Process each mushroom instance
        for box in bounding_boxes:
            # Extract bounding box
            #bbox = ann['bbox']  # Format: [x_min, y_min, width, height]
            #x_min, y_min, width_bbox, height_bbox = bbox
            x_min = box.xmin#int(x_min)
            y_min = box.ymin# int(y_min)
            #width_bbox = int(width_bbox)
            #height_bbox = int(height_bbox)
            x_max = box.xmax# + width_bbox
            y_max =box.ymax # + height_bbox

            # Handle cases where the bounding box is out of image bounds
            if x_min < 0 or y_min < 0 or x_max > width or y_max > height:
                print("Bounding box out of image bounds for box ID:", box.class_id)
                continue

            # Get indices within the bounding box
            bbox_indices = ((x >= x_min) & (x < x_max) & (y >= y_min) & (y < y_max))

            # Get the points and colors within the bounding box
            X_bbox = X[bbox_indices]
            Y_bbox = Y[bbox_indices]
            Z_bbox = Z[bbox_indices]
            points_bbox = np.vstack((X_bbox, Y_bbox, Z_bbox)).T

            if points_bbox.shape[0] < 3:
                print("Not enough valid points for plane fitting for annotation ID:", box.class_id)
                continue

            # Fit a plane to the 3D points
            # Compute the centroid
            centroid = np.mean(points_bbox, axis=0)
            # Center the points
            points_centered = points_bbox - centroid
            # Compute covariance matrix
            H_matrix = np.dot(points_centered.T, points_centered)
            # Singular Value Decomposition
            _, _, Vt = np.linalg.svd(H_matrix)
            # Normal vector is the last row of Vt
            normal_vector = Vt[-1, :]
            # Ensure normal vector points towards the camera (positive Z)
            if normal_vector[2] > 0:
                normal_vector = -normal_vector

            # Compute rotation matrix
            z_axis = np.array([0, 0, 1])
            rotation_axis = np.cross(z_axis, normal_vector)
            rotation_axis_norm = np.linalg.norm(rotation_axis)
            if rotation_axis_norm < 1e-6:
                # The normal vector is aligned with the z-axis
                rotation_matrix = np.eye(3)
            else:
                rotation_axis = rotation_axis / rotation_axis_norm
                angle = np.arccos(np.clip(np.dot(z_axis, normal_vector), -1.0, 1.0))
                # Rodrigues' rotation formula
                K = np.array([[0, -rotation_axis[2], rotation_axis[1]],
                                [rotation_axis[2], 0, -rotation_axis[0]],
                                [-rotation_axis[1], rotation_axis[0], 0]])
                rotation_matrix = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)

            # Convert rotation matrix to Euler angles (XYZ convention)
            r = R.from_matrix(rotation_matrix)
            euler_angles = r.as_euler('xyz', degrees=True)

            # Store rotation results
            rotation_results.append({
                'annotation_id': 1,
                'rotation_x': euler_angles[0],
                'rotation_y': euler_angles[1],
                'rotation_z': euler_angles[2],
                'centroid': centroid,
                'normal_vector': normal_vector
            })
            print(f"Rotation: {euler_angles}")

        # Show the plot
        #plotter.show()
    def imageDepthCallback(self, data):
        global bboxes_msg,result_img_rgb,i,points_xyz_rgb,global_points_xyz_rgb_list
        BIT_MOVE_16 = 2**16
        BIT_MOVE_8 = 2**8
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
  
            # pick one pixel among all the pixels with the closest range:
            #indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            global_camera_xy=r.get("global_camera_xy")
            camera_xy=global_camera_xy.split(",")
            mode=r.get("mode")=="camera_ready"
            #logger.info("in  imageDepthCallback camera_xy:{}, {},cv_image".format(camera_xy[0],camera_xy[1]))
            rows=480#self.intrinsics.width 
            cols=848#self.intrinsics.height

            if 1: #bboxes_msg is not None and len(bboxes_msg.bounding_boxes)>0  and  self.intrinsics is not None:
                #self.pose_estimation(cv_image,result_img_rgb,bboxes_msg.bounding_boxes)
                if((result_img_rgb is not None) and (cv_image is not None) and (self.intrinsics is not None)):
                    logger.info(result_img_rgb.shape)
                    logger.info(cv_image.shape)
                    result_img_rgb = cv2.resize(result_img_rgb, (848,480))                   
                    col, row = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
                    row = row.astype(float)
                    col = col.astype(float)
                    z = cv_image
                    logger.info("z:{}".format(z.shape))
                    x =  z * (col[:,:]- self.intrinsics.ppx) / self.intrinsics.fx
                    y =  z * (row[:,:]- self.intrinsics.ppy) / self.intrinsics.fy

                    logger.info("x,y:{},{}".format(x.shape,y.shape))
                    red =result_img_rgb[:,:,0]
                    logger.info(":,red{}".format(red.shape))
                    green = result_img_rgb[:,:,1]
                    blue =result_img_rgb[:,:,2]
                    pointsxyzrgb = np.dstack((x, y, z, red, green, blue))
                    logger.info(":,pointsxyzrgb{}".format((pointsxyzrgb.shape)))
                    pointsxyzrgb = pointsxyzrgb.reshape(-1,6)
                    pointsxyzrgb=pointsxyzrgb.T  # important to transpose
                    logger.info("new_points_xyz_rgb{}".format((pointsxyzrgb.shape)))
                    Y = 0.2126 *pointsxyzrgb[3]+ 0.7152*pointsxyzrgb[4]+ 0.0722*pointsxyzrgb[5]
                    rgb = pointsxyzrgb[3]*BIT_MOVE_16+ pointsxyzrgb[4]*BIT_MOVE_8+pointsxyzrgb[5]
                    logger.info("Y{}".format((Y.shape)))
                    new_points_xyz_rgb=list(zip(pointsxyzrgb[0],pointsxyzrgb[1],pointsxyzrgb[2],pointsxyzrgb[3],pointsxyzrgb[4],pointsxyzrgb[5]))
                    #logger.info("new_points_xyz_rgb{}".format((new_points_xyz_rgb)))
                    new_points_xyz_rgb_list=np.array(new_points_xyz_rgb,  dtype=[
                                                                                ('x', np.float32),
                                                                                ('y', np.float32),
                                                                                ('z', np.float32),
                                                                                #('rgb', np.uint32),
                                                                                ('r', np.uint8),
                                                                                ('g', np.uint8),
                                                                                ('b', np.uint8)
                                                                                ])

                    logger.info("new_points_xyz_rgb_list{}".format(len(new_points_xyz_rgb_list)))

                    cloud_msg=rnp.msgify(PointCloud2, new_points_xyz_rgb_list)
                    cloud_msg.header=Header()
                    cloud_msg.header.stamp = self.get_clock().now().to_msg()#seconds #rospy.Time(t_us/10000000.0)
                    cloud_msg.header.frame_id = "camera_link"
                    self.pub_pointclouds.publish(cloud_msg)


                    logger.info("count==true?{}".format(self.count=="0"))
                    if self.count=="0":
                        self.create_point_cloud_file2(new_points_xyz_rgb_list,"new_mushroom_total.ply")
                        msg=String()
                        msg.data="1"
                        self.pub_count2.publish(msg)

                    #self.i=self.i+1

                bboxes_msg=None

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        
    def MoveXYZCallback(self, data):
        global bboxes_msg
        logger.info("/move/x {}".format(data.data))
        xyz=data.data.split(";")
        x=xyz[0]
        y=xyz[1]
        z=xyz[2]
        logger.info(" local cord  to camera:{}, {}".format(x,y))
        try:

            global_camera_xy=r.get("global_camera_xy")
            camera_xy=global_camera_xy.split(",")
            logger.info("camera_xy:{}, {}".format(camera_xy[0],camera_xy[1]))
            if r.get("mode")=="camera_ready":
                #r.set("mode","pickup_ready")                            
                    obj=str(int(float(camera_xy[0]))+x)+","+str(int(float(camera_xy[1]))+y)+","+str(int(float(z)))
                    #logger.info(line)
                    if 1:
                        r.hset("detections", "-1", obj)
                        r.lpush("queue","-1")
                        r.hset("detections_history", "-1", obj) 
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
    
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return
    def boxes_callback(self, data):
        logger.info("boxes_callback")
        #global bounding_boxes
        #bounding_boxes=None
        #if 1:#r.get("mode")=="camera_ready":
         #   bounding_boxes=data.bounding_boxes
            #r.set("mode","pickup_ready")
        #for box in bounding_boxes:
        #    logger.info(" boxes_callback probability,%4.2f,%s,x=%4.2f,y=%4.2f",box.probability,box.class_id,(box.xmin+box.xmax)/2,(box.ymin+box.ymax)/2)


def ros2_thread(node):
    print('entering ros2 thread')
    rclpy.spin(node)
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler
    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)


#rclpy.init(args=None)
#ros2_node = MovePublisher()
#app = Flask(__name__)
#threading.Thread(target=ros2_thread, args=[ros2_node]).start()
#prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

def ros_main(args = None):
        rclpy.init(args=args)
        ros_class = yolox_ros()

        try:
            rclpy.spin(ros_class)
        except KeyboardInterrupt:
            pass
        finally:
            ros_class.destroy_node()
            rclpy.shutdown()
             
    #rclpy.init(args=None)
    #ros_class = yolox_ros()
    ##app = Flask(__name__)
    #threading.Thread(target=ros2_thread, args=[ros_class]).start()
    #prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

if __name__ == '__main__':
    ros_main()
    app.run(host='0.0.0.0', threaded=True,port='5001')
    logger.info("after app run()")


