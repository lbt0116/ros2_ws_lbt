//
// Created by lbt on 24-12-11.
//

#include "robot_software/robot_interface/MujocoInterface.h"

using namespace std::chrono_literals;

namespace Galileo
{
    MujocoInterface::MujocoInterface(std::shared_ptr<PinocchioInterface> pinocchio_interface):
        Node("interface", rclcpp::NodeOptions().use_intra_process_comms(true))
        , pinocchioInterface_(std::move(pinocchio_interface))
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

        // 创建订阅者

        // 自定义 QoS 配置
        imu_subscriber_.subscribe(this, "imu_data", qos.get_rmw_qos_profile());
        joint_subscriber_.subscribe(this, "joint_states", qos.get_rmw_qos_profile());

        mujocoSub_ = this->create_subscription<custom_msgs::msg::MujocoMsg>(
            "mujoco_msg", qos, std::bind(&MujocoInterface::sub_mujoco_callback, this, std::placeholders::_1));

        // 创建近似时间同步器，队列大小为10，允许最大时间偏差为1ms
        sync_policy_ = std::make_shared<SyncPolicy>(3);
        sync_policy_->setMaxIntervalDuration(1ms); // 允许最大同步偏差为 1ms


        synchronizer_ = std::make_shared<Synchronizer>(SyncPolicy(*sync_policy_), imu_subscriber_,
                                                       joint_subscriber_);

        // 注册同步回调函数
        synchronizer_->registerCallback(std::bind(&MujocoInterface::sub_sensor_callback, this, std::placeholders::_1,
                                                  std::placeholders::_2));

        // synchronizer_->registerCallback(
        //     [this](const sensor_msgs::msg::Imu::SharedPtr imu_msg,
        //            const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
        //     {
        //         this->sub_sensor_callback(imu_msg, joint_state_msg);
        //     });
    }


    void MujocoInterface::sub_mujoco_callback(const custom_msgs::msg::MujocoMsg::SharedPtr msg)
    {
        pinocchioInterface_->set_mujoco_msg(msg);
        RCLCPP_INFO(this->get_logger(), "pub contact %.2d %.2d %.2d %.2d", msg->contact_state[0], msg->contact_state[1],
                    msg->contact_state[2], msg->contact_state[3]);
    }

    void MujocoInterface::sub_sensor_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,
                                              const sensor_msgs::msg::JointState::ConstSharedPtr& joint_state_msg) const
    {
        pinocchioInterface_->set_imu_msg(imu_msg);

        pinocchioInterface_->set_joint_msg(joint_state_msg);

        pinocchioInterface_->update();

        auto msg = custom_msgs::msg::RobotStateMsg();
        eigenToFloat64MultiArray(pinocchioInterface_->legPosBaseInBody, msg.leg_pos_base_in_body);

        // RCLCPP_INFO(this->get_logger(), "pub leg_pos_base_in_body %.2f", msg.leg_pos_base_in_body.data[0]);
    }
} // Galileo
