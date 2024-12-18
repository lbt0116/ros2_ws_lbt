#include "robot_software/robot_interface/UserInterfaceNode.h"

namespace Galileo
{
UserInterfaceNode::UserInterfaceNode()
    : Node("user_interface_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      cmdHandler_(std::make_unique<CmdHandler>())
{
    keyboardSub_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input",
        10,
        std::bind(&UserInterfaceNode::keyboard_callback, this, std::placeholders::_1));

    keyboardPub_ = this->create_publisher<std_msgs::msg::String>("keyboard_output", 10);
}

void Galileo::UserInterfaceNode::keyboard_callback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    // 处理键盘输入
    cmdHandler_->keyboard_callback(msg);
    // 发布键盘输入
    keyboard_publish(msg);
}

void Galileo::UserInterfaceNode::keyboard_publish(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    auto msg_out = std_msgs::msg::String();
    msg_out.data = msg->data;
    keyboardPub_->publish(msg_out);
}
}  // namespace Galileo
