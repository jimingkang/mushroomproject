#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MoveIt2MotionPlanning : public rclcpp::Node {
public:
    MoveIt2MotionPlanning() : Node("moveit2_motion_planning") {
        // Create a MoveGroupInterface for the "manipulator" group
        auto move_group = moveit::planning_interface::MoveGroupInterface(this->shared_from_this(), "scara_arm");

        // Set a target pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.4;
        target_pose.position.y = 0.2;
        target_pose.position.z = 0.0;
        target_pose.orientation.w = 1.0; // Neutral orientation

        move_group.setPoseTarget(target_pose);

        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            RCLCPP_INFO(this->get_logger(), "Motion plan found! Executing...");
            move_group.execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Motion planning failed!");
        }
    }
};

//int main(int argc, char** argv) {
//    rclcpp::init(argc, argv);
//    auto node = std::make_shared<MoveIt2MotionPlanning>();
 //   rclcpp::spin(node);
//    rclcpp::shutdown();
/    return 0;
//}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Next step goes here
// Create the MoveIt MoveGroup Interface
using moveit::planning_interface::MoveGroupInterface;
auto move_group_interface = MoveGroupInterface(node, "scara_arm");

// Set a target Pose
auto const target_pose = []{
  geometry_msgs::msg::Pose msg;
  msg.orientation.w = 1.0;
  msg.position.x = 0.28;
  msg.position.y = -0.2;
  msg.position.z = 0.5;
  return msg;
}();
move_group_interface.setPoseTarget(target_pose);

// Create a plan to that target pose
auto const [success, plan] = [&move_group_interface]{
  moveit::planning_interface::MoveGroupInterface::Plan msg;
  auto const ok = static_cast<bool>(move_group_interface.plan(msg));
  return std::make_pair(ok, msg);
}();

// Execute the plan
if(success) {
  move_group_interface.execute(plan);
} else {
  RCLCPP_ERROR(logger, "Planning failed!");
}
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

