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
      eskf_(std::make_shared<EskfOnSe3>())
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    statePub_ = this->create_publisher<custom_msgs::msg::RobotStateMsg>("robotstate_msg", qos);

    timer_ =
        this->create_wall_timer(1ms, std::bind(&RobotEstimatorNode::pub_estimation_callback, this));
}

void RobotEstimatorNode::pub_estimation_callback()
{
    auto msg = custom_msgs::msg::RobotStateMsg();
    vec4i phase = {1, 1, 1, 1};  // todo phase
    // eskf_->run(phase);

    statePub_->publish(msg);
}
}  // namespace Galileo
