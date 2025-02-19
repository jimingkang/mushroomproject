#include "scara_hardware/scara_hardware.hpp"
#include <cstddef>
#include <string>
#include <sys/types.h>
#include <thread>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#define YAML false

namespace scara
{

CallbackReturn ScaraHardware::on_init(const hardware_interface::HardwareInfo & info)
{
    // Call the base class implementation
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("scara_hardware_node");
    executor_.add_node(node_);

    // Get the joint names
    vector<double> offsets;
    vector<double> ratios;
#if YAML == false
    //uint count = 7;
    //joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    //motor_names = {"J1", "J2", "J3", "J4", "J5", "J6", "J7"};
    //offsets = {0.0, -1.80, 2.1, 1.57*20, 0.0, -1.57*20, 0.0};
    //ratios = {-1070.0, 1.0, 1.0, -20.0, -20.0, -20.0, 108.0};
    
    uint count = 4;
    joint_names = {"joint1", "joint2", "joint3", "joint4"};
    motor_names = {"J1", "J2", "J3", "J4"};
    offsets = {0.0, -1.80, 2.1, 1.57*20};
    ratios = {-1070.0, 1.0, 1.0, -20.0};
#else
    // uint count = node_->declare_parameter("joint_count", 0);
    // joint_names = node_->declare_parameter("joint_names", joint_names);
    // motor_names = node_->declare_parameter("motor_names", motor_names);
    // offsets = node_->declare_parameter("offsets", offsets);
    // ratios = node_->declare_parameter("ratios", ratios);
#endif

    // Add the joints to the map
    for (size_t i = 0; i < count; i++)
    {
        mj_map.add_joint(motor_names[i], joint_names[i], offsets[i], ratios[i]);
    }

    // get ready for the joint states and commands
    for (const auto& joint : joint_names)
    {
        joint_states[joint] = {0.0, 0.0};
        joint_commands[joint] = {0.0, 0.0};
    }

    // Check if the joint names match
    auto joint_names_set = mj_map.get_joint_names();
    for (auto joint : info.joints)
    {
        if (joint_names_set.find(joint.name) == joint_names_set.end())
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint %s not found in the joint map", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
    }
    
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/hitbot/joint_states", 10, std::bind(&ScaraHardware::jointStateCallback, this, std::placeholders::_1));

    // Create publisher and subscriber
    motor_state_sub_ = node_->create_subscription<device_interface::msg::MotorState>(
        "motor_state", 10, std::bind(&ScaraHardware::motor_state_callback, this, std::placeholders::_1));
    motor_goal_pub_ = node_->create_publisher<device_interface::msg::MotorGoal>("motor_goal", 10);
    std::thread([this]() { executor_.spin(); }).detach();

    // Done
    RCLCPP_INFO(node_->get_logger(), "ScaraHardware initialized.");
    return CallbackReturn::SUCCESS;
}

void ScaraHardware::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Update internal joint positions based on incoming HitBot joint states
        for (size_t i = 0; i < msg->position.size() ; i++) {
            //joint_positions_[i] = msg->position[i];
            
            this->joint_states[joint_names[i]]= {msg->position[i],0};
             RCLCPP_INFO(node_->get_logger(), "msg->positions i=%d,%f\n",i,msg->position[i]);
              RCLCPP_INFO(node_->get_logger(), " this->joint_states=%s\n", this->joint_states[joint_names[i]]]);
            
            
        }
        // RCLCPP_INFO(node_->get_logger(), "msg->positions");
    }
std::vector<hardware_interface::StateInterface> ScaraHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto& [joint, state]: joint_states)
    {
        auto& [pos, vel] = state;
        state_interfaces.emplace_back(joint, "position", &pos);
        state_interfaces.emplace_back(joint, "velocity", &vel);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ScaraHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto& [joint, command]: joint_commands)
    {
        auto& [pos, vel] = command;
        command_interfaces.emplace_back(joint, "position", &pos);
        command_interfaces.emplace_back(joint, "velocity", &vel);
    }
    return command_interfaces;
}

return_type ScaraHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // update the joint states
    // already done in the callback
       for (const auto& joint : joint_names)
    {
      
      // RCLCPP_INFO(node_->get_logger(), "read joint=%s,states=%s\n",joint,this->joint_states[joint]);
    }
 
    return return_type::OK;
}

return_type ScaraHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // publish the joint commands
    device_interface::msg::MotorGoal msg;
    for (const auto& [joint, command]: joint_commands)
    {
        auto [pos, vel] = command;
        auto motor = mj_map.j2m_name(joint);
        msg.motor_id.push_back(motor);
        // Check if pos and vel are both not NaN
        if (!std::isnan(pos) && !std::isnan(vel))
            // If both are not NaN, set vel to NaN
            vel = std::nan("");

        msg.goal_pos.push_back(mj_map.j2m_pos(joint, pos));
        msg.goal_vel.push_back(mj_map.j2m_vel(joint, vel));
        msg.goal_tor.push_back(std::nan(""));
    }
    motor_goal_pub_->publish(msg);
    return return_type::OK;
}

void ScaraHardware::motor_state_callback(const device_interface::msg::MotorState::SharedPtr msg)
{
    // Update the joint states
    for (size_t i = 0; i < msg->motor_id.size(); i++)
    {
        auto& motor = msg->motor_id[i];
        if (std::find(motor_names.begin(), motor_names.end(), motor) == motor_names.end()) continue;
        auto joint = mj_map.m2j_name(motor);
        auto joint_pos = mj_map.m2j_pos(motor, msg->present_pos[i]);
        auto joint_vel = mj_map.m2j_vel(motor, msg->present_vel[i]);
        this->joint_states[joint] = {joint_pos, joint_vel};
    }
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(scara::ScaraHardware, hardware_interface::SystemInterface)
