#include "robot_software/robot_controller/nmpc_controller/GeometricNmpcNode.h"

namespace Galileo
{
GeometricNmpcNode::GeometricNmpcNode()
    : Node("geometric_nmpc_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      dataCenter_(DataCenter::getInstance())
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    nmpc_.declare_and_get_parameters(this);
    publisher_ = this->create_publisher<custom_msgs::msg::ActuatorCmds>("actuators_cmds", qos);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&GeometricNmpcNode::publish_commands, this));

    // 订阅 /parameter_events 主题
    parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10, std::bind(&GeometricNmpcNode::on_parameter_event, this, std::placeholders::_1));
}

void GeometricNmpcNode::on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    nmpc_.set_parameters(event);
}

void GeometricNmpcNode::publish_commands()
{
    // 发布命令
    // nmpc_.run();
}

}  // namespace Galileo