#pragma once

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "rclcpp/rclcpp.hpp"

namespace Galileo
{
class RobotControllerNode : public rclcpp::Node
{
public:
    RobotControllerNode();

private:
    void publish_commands();

    std::vector<std::string> joint_names_;
    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace Galileo