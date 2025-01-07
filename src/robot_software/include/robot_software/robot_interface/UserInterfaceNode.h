#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "custom_msgs/msg/to_sim_msg.hpp"
#include "custom_msgs/msg/to_ui_msg.hpp"
#include "robot_software/robot_utils/communication/KeyboardCmdHandler.hpp"
#include "robot_software/robot_utils/communication/UserDataHandler.hpp"
namespace Galileo
{
class UserInterfaceNode : public rclcpp::Node
{
public:
    UserInterfaceNode();
    ~UserInterfaceNode() override = default;

private:
    // 键盘订阅
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboardSub_;
    // 键盘回调
    void keyboard_callback(const std_msgs::msg::String::ConstSharedPtr& msg);
    // 键盘发布
    void keyboard_publish(const std_msgs::msg::String::ConstSharedPtr& msg);
    // 用户交互命令处理
    std::unique_ptr<KeyboardCmdHandler> keyboardCmdHandler_;
    // 用户交互数据处理
    std::unique_ptr<UserDataHandler> userDataHandler_;
    // 用户交互数据发布
    rclcpp::Publisher<custom_msgs::msg::ToUiMsg>::SharedPtr userDataPub_;
    // 用户交互数据更新定时器
    rclcpp::TimerBase::SharedPtr userDataTimer_;
    // 仿真开始发布
    rclcpp::Publisher<custom_msgs::msg::ToSimMsg>::SharedPtr simulationPub_;
    // 仿真开始定时器
    rclcpp::TimerBase::SharedPtr simulationTimer_;
    // 用户交互数据更新回调
    void user_data_callback();
    // 仿真开始回调
    void simulation_callback();
};
}  // namespace Galileo