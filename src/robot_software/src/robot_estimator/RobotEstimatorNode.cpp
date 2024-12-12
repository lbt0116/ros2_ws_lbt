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
        , pinocchioInterface_(std::move(pinocchio_interface)),
        eskf_(std::make_shared<EskfOnSe3>(std::move(pinocchio_interface)))
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);


        statePub_ = this->create_publisher<custom_msgs::msg::RobotStateMsg>("robotstate_msg", qos);

        timer_ = this->create_wall_timer(1ms, std::bind(&RobotEstimatorNode::pub_estimation_callback, this));
    }


    void RobotEstimatorNode::pub_estimation_callback()
    {
        auto msg = custom_msgs::msg::RobotStateMsg();
        vec4i phase = {1, 1, 1, 1}; // todo phase
        // eskf_->run(phase);

        eigenToFloat64MultiArray(pinocchioInterface_->legPosBaseInBody, msg.leg_pos_base_in_body);
        statePub_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "pub leg_pos_base_in_body %.2f %.2f %.2f", eskf_->p(0), eskf_->p(1),
        //             eskf_->p(2)) ;
        // msg.
    }
} // Galileo
