//
// Created by lbt on 24-12-11.
//

#ifndef MUJOCOINTERFACE_H
#define MUJOCOINTERFACE_H
#include <Eigen/Dense>
#include <memory>  // 用于智能指针

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_interface/PinocchioInterfaceNode.h"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/MatrixTypes.h"
#include "robot_software/robot_utils/UtilFunc.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace Galileo
{
class MujocoInterfaceNode : public rclcpp::Node
{
public:
    MujocoInterfaceNode();  //
    ~MujocoInterfaceNode() override = default;

private:
    void sub_mujoco_callback(const custom_msgs::msg::MujocoMsg::SharedPtr msg);
    void sub_sensor_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
                             const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg) const;

    // void pub_estimation_callback();

    rclcpp::Subscription<custom_msgs::msg::MujocoMsg>::SharedPtr mujocoSub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr triggerPub_;

    // 定义同步策略和同步器
    using SyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, sensor_msgs::msg::JointState>;
    using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

    // 订阅者
    // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> imu_subscriber_;
    // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>> joint_subscriber_;

    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_subscriber_;
    message_filters::Subscriber<sensor_msgs::msg::JointState> joint_subscriber_;

    // 同步策略和同步器
    std::shared_ptr<SyncPolicy> sync_policy_;
    std::shared_ptr<Synchronizer> synchronizer_;
    DataCenter& dataCenter;
};
}  // namespace Galileo

#endif  // MUJOCOINTERFACE_H
