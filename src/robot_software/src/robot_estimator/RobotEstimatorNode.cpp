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
      eskf_(std::make_shared<EskfOnSe3>()),
      dataCenter(DataCenter::getInstance())
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(
            &RobotEstimatorNode::trigger_callback, this, std::placeholders::_1));  // 订阅触发信号
}

void RobotEstimatorNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    RCLCPP_INFO(this->get_logger(), "Trigger received: %d", msg->data);
    // eskf_->run(msg->data);
}
}  // namespace Galileo
