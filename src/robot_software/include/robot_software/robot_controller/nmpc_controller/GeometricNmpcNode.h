#pragma once

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_controller/nmpc_controller/GeometricNmpc.h"
#include "robot_software/robot_utils/DataCenter.hpp"
namespace Galileo
{
class GeometricNmpcNode : public rclcpp::Node
{
public:
    GeometricNmpcNode();
    ~GeometricNmpcNode() override = default;

private:
    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    void publish_commands();

    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    DataCenter& dataCenter_;

    GeometricNmpc nmpc_;
};
}  // namespace Galileo