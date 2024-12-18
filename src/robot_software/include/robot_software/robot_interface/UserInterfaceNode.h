#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_software/robot_utils/CmdHandler.hpp"

namespace Galileo
{
class UserInterfaceNode : public rclcpp::Node
{
public:
    UserInterfaceNode();
    ~UserInterfaceNode() override = default;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboardSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keyboardPub_;
    void keyboard_callback(const std_msgs::msg::String::ConstSharedPtr& msg);
    void keyboard_publish(const std_msgs::msg::String::ConstSharedPtr& msg);
    std::unique_ptr<CmdHandler> cmdHandler_;
};
}  // namespace Galileo
