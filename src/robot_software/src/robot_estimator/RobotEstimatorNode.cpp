//
// Created by lbt on 24-12-8.
//

#include <utility>

#include "robot_software/robot_estimator/RobotEstimatorNode.h"
using namespace std::chrono_literals;

namespace Galileo
{
    RobotEstimatorNode::RobotEstimatorNode(std::shared_ptr<PinocchioInterface> pinocchio_interface):
        Node("estimate_node", rclcpp::NodeOptions().use_intra_process_comms(true))
        , pinocchioInterface_(std::move(pinocchio_interface))
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);


        statePub_ = this->create_publisher<custom_msgs::msg::RobotStateMsg>("robotstate_msg", qos);

        timer_ = this->create_wall_timer(1ms, std::bind(&RobotEstimatorNode::pub_estimation_callback, this));
    }


    void RobotEstimatorNode::pub_estimation_callback()
    {
        auto msg = custom_msgs::msg::RobotStateMsg();
        eigenToFloat64MultiArray(pinocchioInterface_->legPosBaseInBody, msg.leg_pos_base_in_body);
        statePub_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "pub leg_pos_base_in_body %.2f", msg.leg_pos_base_in_body.data[0]);
        // msg.
    }
} // Galileo
