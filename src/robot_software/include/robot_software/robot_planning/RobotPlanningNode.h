#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "robot_software/robot_planning/RobotBasePlanner.h"
#include "robot_software/robot_planning/RobotJointPlanner.h"
#include "robot_software/robot_planning/RobotLegPlanner.h"
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

    std::unique_ptr<RobotLegPlanner> legPlanner_;
    std::unique_ptr<RobotBasePlanner> basePlanner_;
    std::unique_ptr<RobotJointPlanner> jointPlanner_;
};
}  // namespace Galileo
