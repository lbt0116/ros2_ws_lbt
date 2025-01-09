#pragma once

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/low_cmd.hpp"
#include "custom_msgs/msg/low_state.hpp"
#include "custom_msgs/msg/motor_cmd.hpp"
#include "custom_msgs/msg/motor_state.hpp"
#include "robot_hardware/motor/interface.h"
#include "robot_hardware/motor/udp.h"
#include "sensor_msgs/msg/joint_state.hpp"
class MotorDriver
{
public:
    MotorDriver();
    ~MotorDriver();
    custom_msgs::msg::LowState joint_pub_callback();
    void joint_sub_callback(const custom_msgs::msg::LowCmd::SharedPtr msg);

private:
    MotorCmd Convert(const custom_msgs::msg::MotorCmd& msg);
    custom_msgs::msg::MotorState Convert(const MotorState& data);

    // 单模块通信测试
    Interface interface;
};
