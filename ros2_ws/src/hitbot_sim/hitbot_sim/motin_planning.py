import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class MotionPlanClient(Node):
    def __init__(self):
        super().__init__('motion_plan_client')
        self.client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self):
        request = GetMotionPlan.Request()

        # Set up the MotionPlanRequest
        motion_plan_request = MotionPlanRequest()

        # Specify the planning group
        motion_plan_request.group_name = "scara_arm"  # Replace with your planning group

        # Set up the goal constraints
        goal_constraints = Constraints()

        # Example: Add a joint constraint (optional)
        #joint_constraint = JointConstraint()
        #joint_constraint.joint_name = "joint_1"  # Replace with your joint name
        #joint_constraint.position = 0.5  # Target position
        #joint_constraint.tolerance_above = 0.1
        #joint_constraint.tolerance_below = 0.1
        #joint_constraint.weight = 1.0
        #goal_constraints.joint_constraints.append(joint_constraint)

        # Example: Add a position constraint (optional)
        position_constraint = PositionConstraint()
        position_constraint.header = Header(frame_id="base_link")  # Replace with your frame
        position_constraint.link_name = "scara_hand"  # Replace with your end effector link
        position_constraint.target_point_offset.x = 0.4
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0
        position_constraint.constraint_region.primitive_poses.append(PoseStamped())  # Add target pose
        position_constraint.weight = 1.0
        goal_constraints.position_constraints.append(position_constraint)

        # Add the goal constraints to the request
        motion_plan_request.goal_constraints.append(goal_constraints)

        # Set the request in the service call
        request.motion_plan_request = motion_plan_request

        # Call the service
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    client = MotionPlanClient()
    response = client.send_request()

    if response.error_code.val == response.error_code.SUCCESS:
        client.get_logger().info('Motion plan succeeded!')
        # Access the planned trajectory
        planned_trajectory = response.motion_plan.trajectory
        client.get_logger().info(f'Planned trajectory: {planned_trajectory}')
    else:
        client.get_logger().error('Motion plan failed!')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()