#pragma once

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "proxsuite-nlp/constraint-set.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_controller/nmpc_controller/OptProblemBase.h"
#include "robot_software/robot_utils/DataCenter.hpp"

namespace Galileo
{
class NmpcDdpCtrlNode : public rclcpp::Node
{
public:
    NmpcDdpCtrlNode();
    ~NmpcDdpCtrlNode() override = default;

private:
    void declare_and_get_parameters(rclcpp::Node* node);
    void set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr publisher_;

public:
    void publish_commands();
};
}  // namespace Galileo