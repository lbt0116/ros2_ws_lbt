//
// Created by lbt on 24-12-8.
//

#include "robot_software/robot_estimator/RobotEstimatorNode.h"

#include <utility>
using namespace std::chrono_literals;

namespace Galileo
{
RobotEstimatorNode::RobotEstimatorNode()
    : Node("estimate_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      eskf_(std::make_unique<EskfOnSe3>()),
      lkf_(std::make_unique<LinearKalmanFilter>()),
      dataCenter(DataCenter::getInstance())
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&RobotEstimatorNode::trigger_callback, this, std::placeholders::_1));  // 订阅触发信号
    RCLCPP_INFO(this->get_logger(), "RobotEstimatorNode initialized");
}

void RobotEstimatorNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    // eskf_->run(msg->data);
    lkf_->run();
    // RCLCPP_INFO(this->get_logger(),
    //             "lkf pos, %.4f, %.4f, %.4f",
    //             lkf_->getPosition().x(),
    //             lkf_->getPosition().y(),
    //             lkf_->getPosition().z());
    // RCLCPP_INFO(this->get_logger(),
    //             "lkf v: %.4f, %.4f, %.4f",
    //             lkf_->getVelocity().x(),
    //             lkf_->getVelocity().y(),
    //             lkf_->getVelocity().z());
    // RCLCPP_INFO(this->get_logger(),
    //             "lkf q: %.4f, %.4f, %.4f",
    //             lkf_->getAngle().x(),
    //             lkf_->getAngle().y(),
    //             lkf_->getAngle().z());
}
}  // namespace Galileo
