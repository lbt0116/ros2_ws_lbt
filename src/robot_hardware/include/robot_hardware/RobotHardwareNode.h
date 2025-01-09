#pragma once

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_hardware/imu/imu_yis320.h"
#include "robot_hardware/motor/motor_driver.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
class RobotHardwareNode : public rclcpp::Node
{
public:
    RobotHardwareNode();
    ~RobotHardwareNode();

    void sensor_pub_callback();
    void joint_sub_callback(const custom_msgs::msg::LowCmd::SharedPtr msg);

private:
    std::shared_ptr<ImuYIS320> imu_;
    std::shared_ptr<MotorDriver> motor_driver_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<custom_msgs::msg::LowState>::SharedPtr joint_pub_;
    rclcpp::Subscription<custom_msgs::msg::LowCmd>::SharedPtr joint_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};
