#include "robot_software/robot_interface/UserInterfaceNode.h"

namespace Galileo
{
UserInterfaceNode::UserInterfaceNode()
    : Node("user_interface_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      keyboardCmdHandler_(std::make_unique<KeyboardCmdHandler>()),
      userDataHandler_(std::make_unique<UserDataHandler>())
{
    keyboardSub_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input",
        10,
        std::bind(&UserInterfaceNode::keyboard_callback, this, std::placeholders::_1));

    userDataPub_ = this->create_publisher<custom_msgs::msg::ToUiMsg>("user_data_output", 10);
    userDataTimer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                             std::bind(&UserInterfaceNode::userDataCallback, this));
}

void Galileo::UserInterfaceNode::keyboard_callback(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    // 处理键盘输入
    keyboardCmdHandler_->keyboard_callback(msg);
}

void Galileo::UserInterfaceNode::userDataCallback()
{
    // 更新用户交互数据
    custom_msgs::msg::ToUiMsg msg = userDataHandler_->updateUserData();
    userDataPub_->publish(msg);
}
}  // namespace Galileo