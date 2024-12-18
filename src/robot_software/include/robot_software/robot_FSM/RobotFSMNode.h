#ifndef ROBOT_FSM_NODE_H
#define ROBOT_FSM_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include "robot_software/robot_FSM/FiniteStateMachine.h"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "std_msgs/msg/bool.hpp"
namespace Galileo
{

class RobotFSMNode : public rclcpp::Node
{
public:
    RobotFSMNode();
    ~RobotFSMNode() override = default;

private:
    // 订阅触发信号
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr triggerSub_;

    // 触发信号回调函数
    void trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg);

    // 数据中心
    DataCenter& dataCenter;
};

}  // namespace Galileo

#endif  // ROBOT_FSM_NODE_H