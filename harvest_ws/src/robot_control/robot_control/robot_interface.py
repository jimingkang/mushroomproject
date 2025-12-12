import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .Hitbot.RobotWrapper import ScaraRobot
from .Hitbot.iksolver import OldSolver
import numpy as np
from std_msgs.msg import String,Int32
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from std_srvs.srv import Trigger
from .Hitbot.gripperclient import ServiceClient
import  redis
import math
import time

from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
broker="172.23.66.117"
redis_server='172.23.248.33'
global_z=0

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)
class Robot(Node, ScaraRobot):
    def __init__(self):
        Node.__init__(self, 'robot_control_node')  # Call Node's __init__
        ScaraRobot.__init__(self)

        # Subscribers
        #self.joint_state_subscriber = self.create_subscription(
        #    JointState, "joint_states", self.update_joint_states, 10
        #)
        # Timer
        #timer_period = 0.5
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publisher
        self.robot_move_status = self.create_publisher(String, "move_status", 10)



        # Store joint positions
        self.joint_position = None

        #jimmy add
        self.joint_names = ["joint0","joint1", "joint2", "joint3"] 
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_command_sub = self.create_subscription(DisplayTrajectory,"/rrt_path",self.joint_command_callback,10)
        #self.client_node = ServiceClient()
        self.goal_pose_pub=self.create_publisher(PoseStamped,"/goal_pose",10)
        self.bounding_boxes_sub = self.create_subscription(String,"/d435/yolox/bounding_boxes",self.bounding_boxes_callback, 2)
        self.adj_bounding_boxes_sub = self.create_subscription(String,"/d405/yolox/adj_bounding_boxes",self.adj_bounding_boxes_callback, 10)
        self.rrt_done_sub = self.create_subscription(String,"/rrt_done",self.rrtdone_callback, 10)
        self.solver=OldSolver()
        r.set("mode","camera_ready")

    def publish_joint_states(self):
        # Get real joint positions from HitBot API (Replace this with actual API calls)
        self.get_scara_param()
        self.wait_stop()
        joint_positions = [self.z,self.angle1*3.14/180,self.angle2*3.14/180,(self.r)*3.14/180]

        # Create JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joint_positions
        # Publish the joint states
        #self.get_logger().info(f"publish_joint_states :joint_positions={joint_positions}")
        self.joint_state_pub.publish(joint_state_msg)

    def normalize_angle(self,angle):
        return math.atan2(math.sin(angle), math.cos(angle))    
    def joint_command_callback(self, msg):
        try:
            positions = []
            #joint_limits = [(-1.2, 0.0), (-1.571, 1.571), (-2.967, 2.967), (-18.850, 18.850)]
            trajs=msg.trajectory[-1].joint_trajectory.points
            #self.get_logger().info(f"joint_command_callback trajectory: {trajs}")
            for i in range(0, len(trajs)):
                waypoints = trajs[i].positions
                self.get_logger().info(f"i={i},waypoints : {waypoints}")
                self.move_joint_radian(waypoints[0],waypoints[1],waypoints[2],30,1)
            self.wait_stop()
            #response = self.client_node.open_send_request()
                #if pos < min_limit or pos > max_limit:
                #    print(f"Position for joint{i} must be between {min_limit} and {max_limit}.")
                #    return self.get_positions_from_user()
                #positions.append(pos)
            #return positions
            for i in reversed(range(len(trajs))):
                wp = trajs[i].positions
                self.get_logger().info(f"backward i={i}, waypoints={wp}")
                self.move_joint_radian(wp[0], wp[1], wp[2], 30, 1)
            self.wait_stop()
            
        except ValueError:
            print("Invalid input. Please enter numerical values.")
        self.get_logger().info(f"Sent joint command to  robot")
        r.set("mode","camera_ready")
          

    def bounding_boxes_callback(self, msg):
        global global_z
        #r.set("gripper_flag", "ready")
        
        if r.get("mode")=="camera_ready":
            mushroom_xyz=msg.data
            mushroom_xyz=msg.data.split(",");
            #mushroom_xyz=r.get("mushroom_xyz")
            if mushroom_xyz is None :
                self.get_logger().info(f"no mushroom_xyz")
                return
            #goal=[int(float(mushroom_xyz[2].strip())),0-int(float(mushroom_xyz[0].strip()))] # this is from yolox_ros
            goal=[float(mushroom_xyz[2].strip()),0-float(mushroom_xyz[0].strip())]
            global_z=float(mushroom_xyz[1].strip())
            self.get_logger().info(f"get mushroom_xyz {mushroom_xyz},goal:{goal}")    
            if abs(goal[0]) >0.8 or abs(goal[1])>0.8 or (abs(goal[0])<0.35 and abs(goal[1])<0.35):
                self.get_logger().info(f"mushroom_xyz out of range:{mushroom_xyz},goal:{goal}")    
                return
            #self.move_robo_angles=self.move(goal)
            #self.plan_and_print_once(x=goal[0],y=goal[1],z=0.1,isPickup=True)

            response=self.solver.get_robot_angle_in_degree(goal)
            if response is not None:
                joint_positions = response
                ret=self.movej_angle(joint_positions[0],joint_positions[1],0,joint_positions[2]-0,80,1)
                self.wait_stop()
                #time.sleep(3) 
            else:
                self.get_logger().error(f'❌ IK computation failed: code={response.error_code.val}')

            r.set("mode", "adjust_ready")
    def adj_bounding_boxes_callback(self, msg):
        global global_z
        if  self.x <0.3:
            return
        if r.get("mode") == "adjust_ready" :
            mushroom_xyz = msg.data
            mushroom_xyz = msg.data.split(",")
            self.get_logger().info(f"adj bounding_boxes_callback in top camera:{mushroom_xyz} ")
            goal = [float(mushroom_xyz[0].strip()), float(mushroom_xyz[1].strip()), float(mushroom_xyz[2].strip())]
            self.get_scara_param()
            dist_incamera=math.sqrt(goal[0]*goal[0]+goal[1]*goal[1])
            self.get_logger().info(f"adj bounding_boxes_callback :dist_incamera:{dist_incamera}")
            if (dist_incamera>0.01 and dist_incamera<0.1):#(abs(self.robot.x - goal[2])>50 or abs(self.robot.y - goal[0])>50):
                #this is for adj from top camera
                #RX=-goal[1]
                #RY=-goal[0]
                #RZ=goal[2]
                RX=goal[0]
                RY=-goal[1]
                RZ=goal[2]
                X=RX*math.cos(math.radians(self.r))-RY*math.sin(math.radians(self.r))
                Y=RX*math.sin(math.radians(self.r))+RY*math.cos(math.radians(self.r))
                self.get_logger().info(f"adj bounding_boxes_callback :XY in  top camera:{X},{Y}")
                self.get_scara_param()
                self.wait_stop()
                ret = self.movej_xyz(self.x +X*1000, self.y +Y*1000, self.z, self.r, 30, 1)
                self.wait_stop()
                if ret>1:
                    self.get_logger().info(f'❌ ajd IK computation by three joints')
                    response=self.solver.get_robot_angle_in_degree([self.x/1000+X,self.y/1000+Y])
                    if response is not None:
                        joint_positions = response
                        ret=self.movej_angle(joint_positions[0],joint_positions[1],0,joint_positions[2]-0,30,1)
                        self.wait_stop() 
                    else:
                        self.get_logger().error(f'❌ ajd IK computation failed: code={response.error_code.val}')

                #r.set("mode", "adjust_ready")
                #return
            if 1:  # ret<2:
                r.set("mode", "adjust_done")
                #response = self.client_node.open_send_request()
                if 1:#response is not None:
                    self.get_scara_param()
                    self.wait_stop()
                    ret = self.movej_xyz(self.x, self.y, -100, self.r, 30, 1)
                    self.wait_stop()
                    self.get_logger().info(f"move in -z, ret :{ret}")
                #response = self.client_node.close_send_request()  # close request
                time.sleep(1)
                if 1:#response is not None:
                    self.get_scara_param()
                    self.wait_stop()
                    self.get_logger().info(f'open 2 for {self.z}')
                    for i in range(3):
                        ret = self.movej_xyz(self.x, self.y, self.z, self.r - 3, 30, 1)
                        self.wait_stop()
                        self.get_scara_param()
                        self.wait_stop()
                        ret = self.movej_xyz(self.x, self.y, self.z, self.r + 3, 30, 1)
                        self.wait_stop()
                        time.sleep(0.5)
                    self.get_scara_param()
                    self.wait_stop()
                    ret = self.movej_xyz(self.x, self.y, 0, self.r, 50, 1)
                    self.wait_stop()
                    time.sleep(1)
  
                #self.get_scara_param()
                #ret=self.movej_xyz(600,0,0,0,50,1)
                #self.wait_stop()

                #method 1: move to safe point and then to home
                #self.get_scara_param()
                #ret=self.movej_xyz(0,400,0,90+90,90,1)
                #self.wait_stop()
                #response = self.client_node.open_send_request()



                #methods 2: move directly to home
                goal_pose=PoseStamped()
                goal_pose.header.stamp=self.get_clock().now().to_msg()
                goal_pose.header.frame_id="base_link"
                goal_pose.pose.position.x=-0.1
                goal_pose.pose.position.y=0.4
                self.goal_pose_pub.publish(goal_pose)



            r.set("mode", "rrt_ready")
            #r.set("mode", "camera_ready")

    def rrtdone_callback(self, msg):
        if r.get("mode") == "rrt_done":
            self.get_logger().info(f"rrt done callback received msg: {msg.data}")
            #response = self.client_node.open_send_request()
            r.set("mode", "camera_ready")
        
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Robot()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()