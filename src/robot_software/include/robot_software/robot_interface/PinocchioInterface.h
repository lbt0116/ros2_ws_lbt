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

    void update();

    robot_state::JointState jointState;
    robot_state::LegState legState;
    robot_state::BaseState baseState;
    robot_constants robotConstants;

    // const robot_state::SensorData sensorData;

    double totalMass;

private:
    class PinocchioInterfaceImpl;
    std::unique_ptr<PinocchioInterfaceImpl> impl_;  // 使用PImpl隐藏实现细节

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr triggerSub_;
    void trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
    {
        RCLCPP_INFO(this->get_logger(), "Trigger received: %d", msg->data);
        update();
    };
    // rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr triggerPub_;
    DataCenter& dataCenter;
};
}  // namespace Galileo

#endif  // PINOCCHIOINTERFACEIMPL_H
