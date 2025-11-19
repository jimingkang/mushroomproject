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
import  redis
import math
import time
broker="172.23.66.117"
redis_server='172.23.248.33'

pool = redis.ConnectionPool(host=redis_server, port=6379, decode_responses=True, password='jimmy')
r = redis.Redis(connection_pool=pool)
class Robot(Node, ScaraRobot):
    def __init__(self):
        Node.__init__(self, 'robot_control_node')  # Call Node's __init__
        ScaraRobot.__init__(self)

        # Subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState, "joint_states", self.update_joint_states, 10
        )

        # Publisher
        self.robot_move_status = self.create_publisher(String, "move_status", 10)

        # Timer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Store joint positions
        self.joint_position = None

        #jimmy add 
        self.goal_pose_pub=self.create_publisher(PoseStamped,"/goal_pose",10)
        self.bounding_boxes_sub = self.create_subscription(String,"/d435/yolox/bounding_boxes",self.bounding_boxes_callback, 2)
        self.adj_bounding_boxes_sub = self.create_subscription(String,"/d405/yolox/adj_bounding_boxes",self.adj_bounding_boxes_callback, 10)
        self.solver=OldSolver()
        r.set("mode","camera_ready")

    def update_joint_states(self, msg):
        # Save joint positions as a dictionary
        self.joint_position = {name: pos for name, pos in zip(msg.name, msg.position)}

    def timer_callback(self):
        if self.joint_position is None:
            return
        
        # Publish "Moving" status
        move_msg = String()
        move_msg.data = "Moving"
        self.robot_move_status.publish(move_msg)
        #self.get_logger().info("Robot status: Moving")

        # Move the robot
        angle1 = self.joint_position.get("Angle1", 0.0)
        angle2 = self.joint_position.get("Angle2", 0.0)
        angle3 = self.joint_position.get("Angle3", 0.0)
        prismatic_z = self.joint_position.get("PrismaticJointZ", 0.0)

        self.move_joint_radian(angle1, angle2, angle3)
        self.move_z(int(prismatic_z * 1000))  # Convert meters to mm if needed

        # Optionally log joint positions
        #self.get_logger().info(f"Joint positions: {self.joint_position}")

        # Publish "Free" status after movement
        free_msg = String()
        free_msg.data = "Free"
        self.robot_move_status.publish(free_msg)
        #self.get_logger().info("Robot status: Free")
        r.set("mode","camera_ready")
    def bounding_boxes_callback(self, msg):
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
            self.get_logger().info(f"get mushroom_xyz {mushroom_xyz},goal:{goal}")    
            if abs(goal[0]) >0.8 or abs(goal[1])>0.8 or (abs(goal[0])<0.35 and abs(goal[1])<0.35):
                self.get_logger().info(f"mushroom_xyz out of range:{mushroom_xyz},goal:{goal}")    
                return
            #self.move_robo_angles=self.move(goal)
            #self.plan_and_print_once(x=goal[0],y=goal[1],z=0.1,isPickup=True)

            response=self.solver.get_robot_angle_in_degree(goal)
            if response is not None:
                joint_positions = response
                ret=self.movej_angle(joint_positions[0],joint_positions[1],0,joint_positions[2]-0,30,1)
                self.wait_stop() 
            else:
                self.get_logger().error(f'❌ IK computation failed: code={response.error_code.val}')

            r.set("mode", "adjust_ready")
    def adj_bounding_boxes_callback(self, msg):
        if r.get("mode") == "adjust_ready":
            mushroom_xyz = msg.data
            mushroom_xyz = msg.data.split(",")
            self.get_logger().info(f"adj bounding_boxes_callback,mushroom_xyz:{mushroom_xyz} ")
            goal = [float(mushroom_xyz[0].strip()), float(mushroom_xyz[1].strip()), float(mushroom_xyz[2].strip())]
            self.get_scara_param()
            dist_incamera=math.sqrt(goal[0]*goal[0]+goal[1]*goal[1])
            if (dist_incamera>0.04 and dist_incamera<0.1):#(abs(self.robot.x - goal[2])>50 or abs(self.robot.y - goal[0])>50):
                #this is for adj from top camera
                #RX=-goal[1]
                #RY=-goal[0]
                #RZ=goal[2]
                RX=goal[0]
                RY=-goal[1]
                RZ=goal[2]
                X=RX*math.cos(math.radians(self.r))-RY*math.sin(math.radians(self.r))
                Y=RX*math.sin(math.radians(self.r))+RY*math.cos(math.radians(self.r))
                self.get_scara_param()
                self.wait_stop()
                ret = self.movej_xyz(self.x +X*1000, self.y +Y*1000, self.z, self.r, 30, 1)
                self.wait_stop()
                if ret>1:
                    response=self.solver.get_robot_angle_in_degree([self.x/1000+X,self.y/1000+Y])
                    if response is not None:
                        joint_positions = response
                        ret=self.movej_angle(joint_positions[0],joint_positions[1],0,joint_positions[2]-0,30,1)
                        self.wait_stop() 
                    else:
                        self.get_logger().error(f'❌ ajd IK computation failed: code={response.error_code.val}')

                #ret_distance=self.offset_plan_and_print_once(x=X,y=Y,z=0.1,isPickup=True)
                self.get_logger().info(f"adj bounding_boxes_callback :{goal},x_offset:{X},y_offset:{Y},dist_incamera:{dist_incamera}")
                r.set("mode", "adjust_ready")
                return
            if 1:  # ret<2:
                r.set("mode", "adjust_done")
                #response = self.client_node.open_send_request()
                # if response is not None:
                self.get_scara_param()
                self.wait_stop()
                ret = self.movej_xyz(self.x, self.y, self.z - 50, self.r, 30, 1)
                self.wait_stop()
                self.get_logger().info(f"move in -z, ret :{ret}")
                #response = self.client_node.send_request()  # close request
                time.sleep(3)
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
                # if response is not None:
                self.get_scara_param()
                self.wait_stop()
                ret = self.movej_xyz(self.x, self.y, 0, self.r, 50, 1)
                self.wait_stop()
                time.sleep(1)
                #method 1: move to safe point and then to home
                #self.get_scara_param()
                #ret=self.movej_xyz(600,0,0,0,50,1)
                #self.wait_stop()
                #self.move_robo_angles=self.move([0.10,0.6])
                #self.get_scara_param()
                #ret=self.movej_xyz(0,400,0,90+90,50,1)
                #self.wait_stop()
                #methods 2: move directly to home
                goal_pose=PoseStamped()
                goal_pose.header.stamp=self.get_clock().now().to_msg()
                goal_pose.header.frame_id="base_link"
                goal_pose.pose.position.x=0.0
                goal_pose.pose.position.y=0.4
                self.goal_pose_pub.publish(goal_pose)
                #response = self.client_node.open_send_request()
                #self.move_robo_angles=self.move(np.array([0.5,0.1]))
                #self.move_robo_angles=[]

            r.set("mode", "camera_ready")

        
def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Robot()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()