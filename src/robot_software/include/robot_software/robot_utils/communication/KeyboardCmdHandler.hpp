#pragma once
#include <rclcpp/rclcpp.hpp>

#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/DataTypes.hpp"
#include "std_msgs/msg/string.hpp"
namespace Galileo
{
class KeyboardCmdHandler
{
public:
    KeyboardCmdHandler()
        : data_center_{DataCenter::getInstance()}
    {
    }
    ~KeyboardCmdHandler(){};
    void keyboard_callback(const std_msgs::msg::String::ConstSharedPtr& msg)
    {
        switch (msg->data[0])
        {                                                     // 键盘输入
            case 'w': user_cmd_.veloCmd(0) += 0.1; break;     // 前进
            case 's': user_cmd_.veloCmd.setZero(); break;     // 停止
            case 'a': user_cmd_.angveloCmd(2) += 0.1; break;  // 逆时针旋转
            case 'd': user_cmd_.angveloCmd(2) -= 0.1; break;  // 顺时针旋转
            case 'q': user_cmd_.veloCmd(1) += 0.1; break;     // 向左移动
            case 'e': user_cmd_.veloCmd(1) -= 0.1; break;     // 向右移动
            case 'b': user_cmd_.veloCmd(0) -= 0.1; break;     // 向后移动

            case 'p': user_cmd_.gaitCmd = 0; break;  // 站立
            case '7': user_cmd_.gaitCmd = 1; break;  // 行走
        }
        data_center_.write(user_cmd_);
    }

private:
    DataCenter& data_center_;

    robot_user_cmd::UserCmd user_cmd_;
};

}  // namespace Galileo
