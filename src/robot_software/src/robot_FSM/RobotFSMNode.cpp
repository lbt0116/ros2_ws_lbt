#include "robot_software/robot_FSM/RobotFSMNode.h"

namespace Galileo
{

RobotFSMNode::RobotFSMNode()
    : Node("robot_fsm_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      dataCenter(DataCenter::getInstance()),
      fsm_(std::make_unique<FiniteStateMachine>())
{
    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&RobotFSMNode::trigger_callback, this, std::placeholders::_1));  // 订阅触发信号

    RCLCPP_INFO(this->get_logger(), "Robot FSM Node has been initialized");
}

void RobotFSMNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    fsm_->update_input();
    fsm_->run();
    fsm_->update_output();
    RCLCPP_INFO(this->get_logger(),
                "legPhase received: %d, %d, %d, %d",
                dataCenter.read<robot_FSM::legState>()->legPhase(0),
                dataCenter.read<robot_FSM::legState>()->legPhase(1),
                dataCenter.read<robot_FSM::legState>()->legPhase(2),
                dataCenter.read<robot_FSM::legState>()->legPhase(3));
}

}  // namespace Galileo
