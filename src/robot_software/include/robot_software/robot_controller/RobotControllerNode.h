#pragma once

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_controller/BallanceController.h"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "std_msgs/msg/bool.hpp"

namespace Galileo
{
class RobotControllerNode : public rclcpp::Node, public std::enable_shared_from_this<RobotControllerNode>
{
public:
    RobotControllerNode();
    ~RobotControllerNode() override = default;

private:
    // 发布关节命令
    void publish_commands();

    // 关节名称
    std::vector<std::string> joint_names_;

    // 发布关节命令
    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr publisher_;

    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;

    // 订阅触发信号
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr triggerSub_;

    // 触发信号回调函数
    void trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg);

    // 数据中心
    DataCenter& dataCenter;

    std::unique_ptr<BallanceController> ballanceController_;

    // 动态参数回调函数
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;  // 订阅者
};
}  // namespace Galileo
