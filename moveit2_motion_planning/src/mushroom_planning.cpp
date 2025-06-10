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
//    return 0;
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
 // msg.orientation.w = 1.0;
  msg.orientation.x = -0.000001;

msg.orientation.y = 0.000001;

msg.orientation.z = -0.000001;

msg.orientation.w = 0.519859;
  msg.position.x = 0.48;
  msg.position.y = 0.0;
  msg.position.z = 0.0;
  return msg;
}();



move_group_interface.setStartStateToCurrentState();
move_group_interface.setPoseTarget(target_pose);
move_group_interface.setStartStateToCurrentState();
move_group_interface.setPlanningTime(10.0);
//move_group_interface.setPlannerId("LBKPIECEkConfigDefault");
move_group_interface.setPlannerId("RRTConnectkConfigDefault");
move_group_interface.setGoalTolerance(0.1);

// Disable collision checking for debugging
//move_group_interface.setPlanningSceneDiff(true);



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

/*
auto move_group = MoveGroupInterface(node, "scara_arm");
        // Get current pose
        geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

        // Define waypoints
        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(start_pose);  // Start pose

        geometry_msgs::msg::Pose target_pose = start_pose;
        target_pose.position.x += 0.1;  // Move 10 cm in X direction
        target_pose.position.y += 0.05; // Move 5 cm up
        waypoints.push_back(target_pose);

        // Compute Cartesian Path
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;  // Disable jump detection
        const double eef_step = 0.01;       // Small step for precise movement

        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if (fraction > 0.9) {
            RCLCPP_INFO(logger, "Cartesian path computed successfully (%.2f%% of requested trajectory)", fraction * 100.0);
            move_group.execute(trajectory);
        } else {
            RCLCPP_WARN(logger, "Only %.2f%% of Cartesian path was planned", fraction * 100.0);
        }
*/	
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}

/*
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_motion_plan.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/header.hpp>

using namespace std::chrono_literals;

//class MotionPlanClient : public rclcpp::Node
class MoveIt2MotionPlanning : public rclcpp::Node
{
public:
MoveIt2MotionPlanning() : Node("moveit2_motion_planning") 
  {
    // Create a client for the /plan_kinematic_path service
    client_ = this->create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");

    // Wait for the service to be available
    while (!client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    // Send the request
    send_request();
  }

  void send_request()
  {
    // Create the request
    auto request = std::make_shared<moveit_msgs::srv::GetMotionPlan::Request>();

    // Set up the MotionPlanRequest
    moveit_msgs::msg::MotionPlanRequest motion_plan_request;

    // Specify the planning group
    motion_plan_request.group_name = "scara_arm";  // Replace with your planning group

    // Set up the goal constraints
    moveit_msgs::msg::Constraints goal_constraints;

    // Example: Add a joint constraint (optional)
    moveit_msgs::msg::JointConstraint joint_constraint;
    //joint_constraint.joint_name = "joint_1";  // Replace with your joint name
    //joint_constraint.position = 0.5;         // Target position
    //joint_constraint.tolerance_above = 0.1;
    //joint_constraint.tolerance_below = 0.1;
    //joint_constraint.weight = 1.0;
    //goal_constraints.joint_constraints.push_back(joint_constraint);

    // Example: Add a position constraint (optional)
    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header = std_msgs::msg::Header();
    position_constraint.header.frame_id = "base_link";  // Replace with your frame
    position_constraint.link_name = "scara_hand";    // Replace with your end effector link
    position_constraint.target_point_offset.x = 0.3;
    position_constraint.target_point_offset.y = 0.0;
    position_constraint.target_point_offset.z = 0.0;
    position_constraint.constraint_region.primitive_poses.push_back(geometry_msgs::msg::PoseStamped());  // Add target pose
    position_constraint.weight = 1.0;
    goal_constraints.position_constraints.push_back(position_constraint);

    // Add the goal constraints to the request
    motion_plan_request.goal_constraints.push_back(goal_constraints);

    // Set the request in the service call
    request->motion_plan_request = motion_plan_request;

    // Call the service asynchronously
    auto future = client_->async_send_request(request);

    // Wait for the response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      //if (response->error_code.val == response->error_code.SUCCESS)
      //{
        RCLCPP_INFO(this->get_logger(), "Motion plan succeeded!");
        // Access the planned trajectory
        //auto planned_trajectory = response->trajectory;
       // RCLCPP_INFO(this->get_logger(), "Planned trajectory has %ld points", planned_trajectory.joint_trajectory.points.size());
      //}
      //else
     // {
        RCLCPP_ERROR(this->get_logger(), "Motion plan failed!");
     // }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service /plan_kinematic_path");
    }
  }

private:
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveIt2MotionPlanning>();
  rclcpp::shutdown();
  return 0;
}
*/
