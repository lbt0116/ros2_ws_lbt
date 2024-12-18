#include "robot_software/robot_interface/UserInterfaceNode.h"

namespace Galileo
{
UserInterfaceNode::UserInterfaceNode()
    : Node("user_interface_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    keyboardSub_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input",
        10,
        std::bind(&UserInterfaceNode::keyboard_callback, this, std::placeholders::_1));

    keyboardPub_ = this->create_publisher<std_msgs::msg::String>("keyboard_output", 10);
}

void Galileo::UserInterfaceNode::keyboard_callback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    switch (msg->data[0])
    {
        case 'w': RCLCPP_INFO(this->get_logger(), "Received key: %s", msg->data.c_str()); break;
        case 's': RCLCPP_INFO(this->get_logger(), "Received key: %s", msg->data.c_str()); break;
        case 'a': RCLCPP_INFO(this->get_logger(), "Received key: %s", msg->data.c_str()); break;
        case 'd': RCLCPP_INFO(this->get_logger(), "Received key: %s", msg->data.c_str()); break;
        case 'q': RCLCPP_INFO(this->get_logger(), "Received key: %s", msg->data.c_str()); break;
        case 'e': RCLCPP_INFO(this->get_logger(), "Received key: %s", msg->data.c_str()); break;
    }
    keyboard_publish(msg);
}

void Galileo::UserInterfaceNode::keyboard_publish(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    auto msg_out = std_msgs::msg::String();
    msg_out.data = msg->data;
    keyboardPub_->publish(msg_out);
}
}  // namespace Galileo
