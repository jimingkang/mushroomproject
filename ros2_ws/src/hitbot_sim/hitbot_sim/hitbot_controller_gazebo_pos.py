import rclpy
import random
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
#from moveit.planning import MoveItPy ,PlanRequestParameters
from geometry_msgs.msg import PoseStamped,Pose
import time

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

class HitbotControllerGazeboPos(Node):
    def __init__(self):
        super().__init__('hitbot_controller_gazebo_pos')
        self.publisher_ = self.create_publisher(JointTrajectory, '/scara_arm_controller/joint_trajectory', 10)
        #self.manual_mode = self.get_input_mode()

        #timer_period = 2  # seconds
        #self.timer = self.create_timer(timer_period, self.publish_trajectory)
        
         # Initialize MoveIt 2 commander
        #self.robot = RobotCommander()
        #self.scene = PlanningSceneInterface()
        #self.group = MoveGroupCommander("scara_arm")  # Change "arm" to your robot’s planning group

        # Set planning parameters
        #self.group.set_planner_id("RRTConnectkConfigDefault")
        #self.group.set_max_velocity_scaling_factor(0.5)
        #self.group.set_max_acceleration_scaling_factor(0.5)
        #self.moveit=MoveItPy(node_name=self.get_name())
        #self.arm=self.moveit.get_plainning_component("scara_arm")
        self._action_client = ActionClient(self, MoveGroup, '/move_group')

    def send_goal(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"

        # Define target pose
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0  # Default orientation

        goal_msg.request.goal_constraints.append(pose)

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"goal_msg:{goal_msg}")
    def plan_and_show_trajectory(self):
        # Get the current position
        #start_pose = self.group.get_current_pose().pose
        start_pose = start_pose = self.arm.get_end_effector_pose()
        # Define a target pose (move slightly in X direction)
        target_pose = Pose()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = start_pose.position.x + 0.03  # Move 10cm forward
        target_pose.pose.position.y = start_pose.position.y
        target_pose.pose.position.z = start_pose.position.z
        target_pose.pose.orientation = start_pose.orientation

        # Set the target pose
        self.group.set_pose_target(target_pose.pose)

        # Plan trajectory
        self.get_logger().info("Planning trajectory...")
        plan = self.group.plan()

        if plan[0]:  # Check if a valid plan was found
            self.get_logger().info("Trajectory planned successfully! Executing...")
            self.get_logger().info(plan)

            # Display trajectory in RViz
            self.group.execute(plan[1], wait=True)

            # Clear target pose
            self.group.clear_pose_targets()
        else:
            self.get_logger().warn("Trajectory planning failed.")

    def get_input_mode(self):
        while True:
            mode = input("Enter 'manual' to input positions manually or 'random' for random positions: ")
            if mode.lower() == 'manual':
                return 'manual'
            elif mode.lower() == 'random':
                return 'random'
            else:
                print("Invalid input. Please enter 'manual' or 'random'.")

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = ''
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        if self.manual_mode == 'manual':
            positions = self.get_positions_from_user()
        else:
            positions = self.get_random_positions()
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        point.accelerations = [0.0, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points.append(point)

        self.publisher_.publish(msg)

    def get_positions_from_user(self):
        try:
            positions = []
            joint_limits = [(-1.2, 0.0), (-1.571, 1.571), (-2.967, 2.967), (-18.850, 18.850)]
            for i in range(1, 5):
                min_limit, max_limit = joint_limits[i-1]
                pos_str = input(f"Enter position for joint{i} ({min_limit}~{max_limit}): ")
                pos = float(pos_str)
                if pos < min_limit or pos > max_limit:
                    print(f"Position for joint{i} must be between {min_limit} and {max_limit}.")
                    return self.get_positions_from_user()
                positions.append(pos)
            return positions
        except ValueError:
            print("Invalid input. Please enter numerical values.")
            return self.get_positions_from_user()

    def get_random_positions(self):
        positions = []
        joint_limits = [(-1.2, 0.0), (-1.571, 1.571), (-2.967, 2.967), (-18.850, 18.850)]
        for limit in joint_limits:
            min_limit, max_limit = limit
            random_pos = random.uniform(min_limit, max_limit)
            positions.append(random_pos)
        return positions

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = HitbotControllerGazeboPos()
    #joint_trajectory_publisher.plan_and_show_trajectory()

    object_coords = [0.4, 0.0, 0.0]  # Replace with actual object detection result
    joint_trajectory_publisher.send_goal(*object_coords)
    rclpy.spin(joint_trajectory_publisher)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
