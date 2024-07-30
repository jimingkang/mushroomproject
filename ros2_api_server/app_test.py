from traitlets import This
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import threading

import redis
from flask import Flask, render_template, Response, jsonify
from threading import Event
from flask import Flask, render_template, request, jsonify
from flask_mqtt import Mqtt

from cv_bridge import CvBridge,CvBridgeError
import cv2
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import numpy as np
import back_point_cloud2 as pc2
import pyrealsense2 as rs
import math
from sensor_msgs.msg import CameraInfo

frame=None
depth_frame=None

# Processing blocks
pc = rs.pointcloud()
#decimate = rs.decimation_filter()
#decimate.set_option(rs.option.filter_magnitude, 2 ** decimate)
colorizer = rs.colorizer()
h=480
w=848
out = np.empty(( h,w, 3), dtype=np.uint8)


class MovePublisher(Node):
    def __init__(self):
        depth_info_topic = '/camera/camera/depth/camera_info'
        super().__init__('test_publisher')
        self.intrinsics = None
        self.publisher = self.create_publisher(String, '/move/x', 1)
        self.sub_info = self.create_subscription(CameraInfo, depth_info_topic, self.imageDepthInfoCallback, 1)
        self.subscription = self.create_subscription(Image,'/yolox/boxes_image',self.chatter_callback,10)
        #self.depth_subscription = self.create_subscription(Image,'/camera/camera/depth/image_rect_raw',self.depth_callback,10)
        #self.pointcloud_subscription = self.create_subscription(PointCloud2,'/ORB_SLAM3/pointclouds',self.pointcloud_callback,10)#
        self.new_pointcloud_subscription = self.create_subscription(Image,'/camera/camera/depth/image_rect_raw',self.new_pointcloud_callback,10)
        self.latest_message = None
        self.bridge = CvBridge()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(visible=False)
        self.o3d_pcd = o3d.geometry.PointCloud()

        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True
    


    def depth_callback(self, msg):
        global frame,depth_frame,out
        depth_frame = self.bridge.imgmsg_to_cv2(msg,"16UC1")
        points = pc.calculate(depth_frame)
        if frame  is not None:
            color_image = np.asanyarray(frame.get_data())
            pc.map_to(frame)
            # Pointcloud data to arrays
            v, t = points.get_vertices(), points.get_texture_coordinates()
            verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
            texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
            if not self.scale or out.shape[:2] == (h, w):
                self.pointcloud(out, verts, texcoords, color_image)
            else:
                tmp = np.zeros((h, w, 3), dtype=np.uint8)
                self.pointcloud(tmp, verts, texcoords, color_image)
                tmp = cv2.resize(tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
                np.putmask(out, tmp > 0, tmp)
            if any(self.mouse_btns):
                self.axes(out, self.view(self.pivot()), self.rotation(), thickness=4)
            
    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return



    def chatter_callback(self, msg):
        global frame
        #print(f'chatter cb received: {msg.data}')
        frame = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        #self.latest_message = msg.data
        #frame = msg.data
    def new_pointcloud_callback(self, msg):
        global frame,depth_frame,out
        depth_frame = self.bridge.imgmsg_to_cv2(msg,"16UC1")
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(frame, depth_frame, convert_rgb_to_intensity = False)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsic)
        self.o3d_pcd= pcd# =o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_as_numpy_array[:, 0:3]))#o3d.io.read_point_cloud("../ros2_ws/src/orb_slam3_ros2/result.pcd")#
        self.vis.add_geometry(self.o3d_pcd)
        self.vis.poll_events()
        self.vis.update_renderer()
        self.vis.capture_screen_image("./save.png")
        o3d.visualization.draw_geometries(self.o3d_pcd,zoom=0.3412, front=[0.4257, -0.2125, -0.8795],lookat=[2.6172, 2.0475, 1.532],up=[-0.0694, -0.9768, 0.2024])
  
    def pointcloud_callback(self, msg):
        global frame
        pcd_as_numpy_array = np.array(list(pc2.read_points(msg)))
        if(len(pcd_as_numpy_array)>0):
            print(pcd_as_numpy_array[1])
            print(pcd_as_numpy_array.shape)
            self.o3d_pcd =o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pcd_as_numpy_array[:, 0:3]))#o3d.io.read_point_cloud("../ros2_ws/src/orb_slam3_ros2/result.pcd")#
            self.vis.add_geometry(self.o3d_pcd)
            self.vis.poll_events()
            self.vis.update_renderer()
            self.vis.capture_screen_image("./save.png")
            o3d.visualization.draw_geometries(self.o3d_pcd,zoom=0.3412, front=[0.4257, -0.2125, -0.8795],lookat=[2.6172, 2.0475, 1.532],up=[-0.0694, -0.9768, 0.2024])
        #print(f'chatter cb received: {msg.data}')
        #pc = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        #pc_list = []
        #for p in pc:
        #    pc_list.append( [p[0],p[1],p[2]] )
        #    print(p)

        #self.latest_message = msg.data
        #frame = msg.data
        
    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)


    
    def project(self,v):
        """project 3d vector array to 2d"""
        h, w = out.shape[:2]
        view_aspect = float(h)/w

        # ignore divide by zero for invalid depth
        with np.errstate(divide='ignore', invalid='ignore'):
            proj = v[:, :-1] / v[:, -1, np.newaxis] * (w*view_aspect, h) + (w/2.0, h/2.0)

        # near clipping
        znear = 0.03
        proj[v[:, 2] < znear] = np.nan
        return proj

    def view(self,v):
        """apply view transformation on vector array"""
        return np.dot(v - self.pivot(), self.rotation()) + self.pivot() - self.translation


    def line3d(self,out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
        """draw a 3d line from pt1 to pt2"""
        p0 = self.project(pt1.reshape(-1, 3))[0]
        p1 = self.project(pt2.reshape(-1, 3))[0]
        if np.isnan(p0).any() or np.isnan(p1).any():
            return
        p0 = tuple(p0.astype(int))
        p1 = tuple(p1.astype(int))
        rect = (0, 0, out.shape[1], out.shape[0])
        inside, p0, p1 = cv2.clipLine(rect, p0, p1)
        if inside:
            cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


    def grid(self,out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
        """draw a grid on xz plane"""
        pos = np.array(pos)
        s = size / float(n)
        s2 = 0.5 * size
        for i in range(0, n+1):
            x = -s2 + i*s
            self.line3d(out, self.view(pos + np.dot((x, 0, -s2), rotation)),self.view(pos + np.dot((x, 0, s2), rotation)), color)
        for i in range(0, n+1):
            z = -s2 + i*s
            self.line3d(out, view(pos + np.dot((-s2, 0, z), rotation)), view(pos + np.dot((s2, 0, z), rotation)), color) # type: ignore

    def axes(self,out, pos, rotation=np.eye(3), size=0.075, thickness=2):
        """draw 3d axes"""
        self.line3d(out, pos, pos +np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
        self.line3d(out, pos, pos +np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
        self.line3d(out, pos, pos +np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)

    def frustum(self,out, intrinsics, color=(0x40, 0x40, 0x40)):
        """draw camera's frustum"""
        orig = self.view([0, 0, 0])
        w, h = intrinsics.width, intrinsics.height
        for d in range(1, 6, 2):
            def get_point(x, y):
                p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
                self.line3d(out, orig, self.view(p), color)
                return p
            top_left = get_point(0, 0)
            top_right = get_point(w, 0)
            bottom_right = get_point(w, h)
            bottom_left = get_point(0, h)

            self.line3d(out, self.view(top_left), self.view(top_right), color)
            self.line3d(out, self.view(top_right), self.view(bottom_right), color)
            self.line3d(out, self.view(bottom_right), self.view(bottom_left), color)
            self.line3d(out, self.view(bottom_left), self.view(top_left), color)
    def publish_message(self,msg):
        self.publisher.publish(msg)


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
#threading.Thread(target=ros2_thread, args=[ros2_node]).start()
#prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)


def main(args = None):

    rclpy.init(args = None)

    node = MovePublisher()
    node.get_logger().info("Hello from my_node")

    try:
        rclpy.spin(node)
    except:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





