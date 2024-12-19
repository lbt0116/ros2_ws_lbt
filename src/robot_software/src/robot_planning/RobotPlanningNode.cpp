#include "robot_software/robot_planning/RobotPlanningNode.h"

namespace Galileo
{

RobotPlanningNode::RobotPlanningNode()
    : Node("robot_planning_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      dataCenter(DataCenter::getInstance()),
      legPlanner_(std::make_unique<RobotLegPlanner>())
{
    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(
            &RobotPlanningNode::trigger_callback, this, std::placeholders::_1));  // 订阅触发信号
}

void RobotPlanningNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    legPlanner_->update_leg_trajectory();
    auto a = dataCenter.read<robot_user_cmd::UserCmd>();
    // RCLCPP_INFO(this->get_logger(),
    //             "legTrajectory z: %.2f %.2f %.2f %.2f",
    //             legPlanner_->legTrajectory.p(2, 0),
    //             legPlanner_->legTrajectory.p(2, 1),
    //             legPlanner_->legTrajectory.p(2, 2),
    //             legPlanner_->legTrajectory.p(2, 3));
    RCLCPP_INFO(this->get_logger(), "isStep: %d", dataCenter.read<robot_FSM::legState>()->isStep);

    RCLCPP_INFO(this->get_logger(),
                "timeswitch: %.4f %.4f %.4f",
                dataCenter.read<robot_target_trajectory::TargetLegTrajectory>()->p(0, 0),
                dataCenter.read<robot_target_trajectory::TargetLegTrajectory>()->p(1, 0),
                dataCenter.read<robot_target_trajectory::TargetLegTrajectory>()->p(2, 0));
}

}  // namespace Galileo