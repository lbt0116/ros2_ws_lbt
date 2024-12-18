#include "robot_software/robot_planning/RobotPlanningNode.h"

namespace Galileo
{

RobotPlanningNode::RobotPlanningNode()
    : Node("robot_planning_node", rclcpp::NodeOptions().use_intra_process_comms(true)),
      dataCenter(DataCenter::getInstance())
{
    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(
            &RobotPlanningNode::trigger_callback, this, std::placeholders::_1));  // 订阅触发信号
}

void RobotPlanningNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    auto a = dataCenter.read<robot_user_cmd::UserCmd>();
    RCLCPP_INFO(this->get_logger(),
                "v received: %.2f %.2f %.2f",
                a->veloCmd(0),
                a->veloCmd(1),
                a->veloCmd(2));
}

}  // namespace Galileo