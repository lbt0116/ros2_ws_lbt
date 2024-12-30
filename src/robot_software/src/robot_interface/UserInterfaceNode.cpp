#include "robot_software/robot_interface/UserInterfaceNode.h"

namespace Galileo
{
UserInterfaceNode::UserInterfaceNode()
    : Node("user_interface_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      keyboardCmdHandler_(std::make_unique<KeyboardCmdHandler>()),
      userDataHandler_(std::make_unique<UserDataHandler>())
{
    keyboardSub_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input", 10, std::bind(&UserInterfaceNode::keyboard_callback, this, std::placeholders::_1));

    userDataPub_ = this->create_publisher<custom_msgs::msg::ToUiMsg>("user_data_output", 10);
    simulationPub_ = this->create_publisher<custom_msgs::msg::ToSimMsg>("simulation", 1);
    userDataTimer_ =
        this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&UserInterfaceNode::userDataCallback, this));
    simulationTimer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                               std::bind(&UserInterfaceNode::simulationCallback, this));
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

void Galileo::UserInterfaceNode::simulationCallback()
{
    custom_msgs::msg::ToSimMsg msg;
    msg.start_simulation = true;
    msg.actuators_name.resize(12);

    auto joint_names_ = {"Abd1Joint",
                         "Hip1Joint",
                         "Knee1Joint",
                         "Abd2Joint",
                         "Hip2Joint",
                         "Knee2Joint",
                         "Abd3Joint",
                         "Hip3Joint",
                         "Knee3Joint",
                         "Abd4Joint",
                         "Hip4Joint",
                         "Knee4Joint"};

    msg.actuators_name.assign(joint_names_.begin(), joint_names_.end());

    msg.initial_joint_positions.resize(12);
    msg.initial_joint_positions[0] = 0.0;
    msg.initial_joint_positions[3] = 0.0;
    msg.initial_joint_positions[6] = 0.0;
    msg.initial_joint_positions[9] = 0.0;

    msg.initial_joint_positions[1] = -3.14 / 4;
    msg.initial_joint_positions[4] = -3.14 / 4;
    msg.initial_joint_positions[7] = -3.14 / 4;
    msg.initial_joint_positions[10] = -3.14 / 4;

    msg.initial_joint_positions[2] = 3.14 / 2;
    msg.initial_joint_positions[5] = 3.14 / 2;
    msg.initial_joint_positions[8] = 3.14 / 2;
    msg.initial_joint_positions[11] = 3.14 / 2;

    simulationPub_->publish(msg);
    simulationTimer_->cancel();
}