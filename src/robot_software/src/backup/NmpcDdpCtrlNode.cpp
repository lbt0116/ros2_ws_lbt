// #include "robot_software/robot_controller/nmpc_controller/NmpcDdpCtrlNode.h"
// namespace Galileo
// {
// NmpcDdpCtrlNode::NmpcDdpCtrlNode()
//     : Node("nmpc_ddp_ctrl_node", rclcpp::NodeOptions().use_intra_process_comms(true))
// {
//     auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
//     publisher_ = this->create_publisher<custom_msgs::msg::ActuatorCmds>("actuator_cmds", qos);
// }

// void NmpcDdpCtrlNode::publish_commands()
// {
//     auto msg = custom_msgs::msg::ActuatorCmds();
//     publisher_->publish(msg);
// }

// void NmpcDdpCtrlNode::declare_and_get_parameters(rclcpp::Node* node)
// {
// }

// void NmpcDdpCtrlNode::set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
// {
// }
// }  // namespace Galileo