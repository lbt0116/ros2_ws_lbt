#include "robot_hardware/RobotHardwareNode.h"

using namespace std::chrono_literals;
RobotHardwareNode::RobotHardwareNode()
    : Node("robot_hardware_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    RCLCPP_INFO(this->get_logger(), "RobotHardwareNode init");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    joint_sub_ = this->create_subscription<custom_msgs::msg::LowCmd>(
        "/actuator_cmds", qos, std::bind(&RobotHardwareNode::joint_sub_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(1ms, std::bind(&RobotHardwareNode::sensor_pub_callback, this));
}

RobotHardwareNode::~RobotHardwareNode()
{
}

void RobotHardwareNode::joint_sub_callback(const custom_msgs::msg::LowCmd::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "RobotHardwareNode joint_callback");
    motor_driver_->joint_sub_callback(msg);
}

void RobotHardwareNode::sensor_pub_callback()
{
    sensor_msgs::msg::Imu imu_msg;
    custom_msgs::msg::LowState joint_state_msg;

    imu_msg = imu_->imu_pub_callback();
    joint_state_msg = motor_driver_->joint_pub_callback();

    imu_pub_->publish(imu_msg);
    joint_pub_->publish(joint_state_msg);

    RCLCPP_INFO(this->get_logger(), "RobotHardwareNode run");
}
