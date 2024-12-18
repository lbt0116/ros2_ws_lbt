#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "robot_software/robot_utils/DataCenter.hpp"

namespace Galileo
{

class RobotPlanningNode : public rclcpp::Node
{
public:
    RobotPlanningNode();
    ~RobotPlanningNode() override = default;

private:
    // 订阅触发信号
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr triggerSub_;

    // 触发信号回调函数
    void trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg);
    DataCenter& dataCenter;
};
}  // namespace Galileo