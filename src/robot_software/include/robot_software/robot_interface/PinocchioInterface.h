//
// Created by lbt on 24-12-9.
//

#ifndef PINOCCHIOINTERFACEIMPL_H
#define PINOCCHIOINTERFACEIMPL_H
#include <custom_msgs/msg/detail/mujoco_msg__struct.hpp>
#include <memory>

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"
#include "custom_msgs/msg/robot_state_msg.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/DataTypes.hpp"
#include "robot_software/robot_utils/MatrixTypes.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace Galileo
{
class PinocchioInterface : public rclcpp::Node
{
public:
    PinocchioInterface();
    ~PinocchioInterface();  // 添加析构函数声明

    // 关节状态
    robot_state::JointState jointState;
    // 腿状态
    robot_state::LegState legState;
    // 基座状态
    robot_state::BaseState baseState;
    // 机器人常量
    robot_constants robotConstants;

private:
    // PImpl 模式
    class PinocchioInterfaceImpl;
    std::unique_ptr<PinocchioInterfaceImpl> impl_;  // 使用PImpl隐藏实现细节

    // 订阅触发信号
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr triggerSub_;

    // 触发信号回调函数
    void trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg);

    // 数据中心
    DataCenter& dataCenter;
};
}  // namespace Galileo

#endif  // PINOCCHIOINTERFACEIMPL_H
