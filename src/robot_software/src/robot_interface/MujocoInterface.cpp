//
// Created by lbt on 24-12-11.
//

#include "robot_software/robot_interface/MujocoInterface.h"

using namespace std::chrono_literals;

namespace Galileo
{
MujocoInterface::MujocoInterface()
    : Node("mujoco_interface", rclcpp::NodeOptions().use_intra_process_comms(true)),
      dataCenter(DataCenter::getInstance())
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    // 创建订阅者

    // 自定义 QoS 配置
    imu_subscriber_.subscribe(this, "imu_data", qos.get_rmw_qos_profile());
    joint_subscriber_.subscribe(this, "joint_states", qos.get_rmw_qos_profile());

    mujocoSub_ = this->create_subscription<custom_msgs::msg::MujocoMsg>(
        "mujoco_msg",
        qos,
        std::bind(&MujocoInterface::sub_mujoco_callback, this, std::placeholders::_1));

    triggerPub_ = this->create_publisher<std_msgs::msg::Bool>("trigger", qos);

    // 创建近似时间同步器，队列大小为10，允许最大时间偏差为1ms
    sync_policy_ = std::make_shared<SyncPolicy>(3);
    sync_policy_->setMaxIntervalDuration(1ms);  // 允许最大同步偏差为 1ms

    synchronizer_ = std::make_shared<Synchronizer>(
        SyncPolicy(*sync_policy_), imu_subscriber_, joint_subscriber_);

    // 注册同步回调函数
    synchronizer_->registerCallback(std::bind(
        &MujocoInterface::sub_sensor_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void MujocoInterface::sub_mujoco_callback(const custom_msgs::msg::MujocoMsg::SharedPtr msg)
{
    robot_state::ContactState contactState;
    contactState.isContact << msg->contact_state[0], msg->contact_state[1], msg->contact_state[2],
        msg->contact_state[3];
    contactState.contactPhase << msg->contact_state[0], msg->contact_state[1],
        msg->contact_state[2], msg->contact_state[3];

    contactState.contactForce = stdVectorToEigen(msg->ground_reaction_force, 3, 4);

    // std::cout << contactState.contactForce << std::endl;

    dataCenter.write(contactState);

    RCLCPP_INFO(this->get_logger(),
                "pub contact %.1d %.1d %.1d %.1d",
                contactState.isContact(0),
                contactState.isContact(1),
                contactState.isContact(2),
                contactState.isContact(3));
}

void MujocoInterface::sub_sensor_callback(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
    const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg) const
{
    robot_state::SensorData sensorData;
    sensorData.imuQuant << imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z,
        imu_msg->orientation.w;

    sensorData.imuAcc << imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z;

    sensorData.imuAngvelo << imu_msg->angular_velocity.x, imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z;

    // 按顺序将 position 数据赋值到矩阵中
    for (int i = 0; i < 12; ++i)
    {
        const int row = i % 3;  // 确定当前数据在矩阵中的行
        const int col = i / 3;  // 确定当前数据在矩阵中的列
        sensorData.jointPosition(row, col) = joint_state_msg->position[i];
        sensorData.jointVelocity(row, col) = joint_state_msg->velocity[i];
        sensorData.jointTorque(row, col) = joint_state_msg->effort[i];
    }

    dataCenter.write(sensorData);

    std_msgs::msg::Bool trigger_msg;
    trigger_msg.data = true;
    triggerPub_->publish(trigger_msg);
}
}  // namespace Galileo
