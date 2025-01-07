#include "robot_software/robot_controller/RobotControllerNode.h"

using namespace std::chrono_literals;

namespace Galileo
{
RobotControllerNode::RobotControllerNode()
    : Node("robot_controller", rclcpp::NodeOptions().use_intra_process_comms(true)),
      dataCenter(DataCenter::getInstance()),
      ballanceController_(std::make_unique<BallanceController>()),
      swingLegController_(std::make_unique<SwingLegController>()),
      jointController_(std::make_unique<JointController>())
{
    // 声明参数
    ballanceController_->declare_and_get_parameters(this);
    swingLegController_->declare_and_get_parameters(this);
    jointController_->declare_and_get_parameters(this);
    //
    // 订阅 /parameter_events 主题
    parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events", 10, std::bind(&RobotControllerNode::on_parameter_event, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DynamicParameterNode initialized and listening for parameter events.");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    publisher_ = this->create_publisher<custom_msgs::msg::ActuatorCmds>("actuators_cmds", qos);
    timer_ = this->create_wall_timer(1ms, std::bind(&RobotControllerNode::publish_commands, this));

    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&RobotControllerNode::trigger_callback, this, std::placeholders::_1));  // 订阅触发信号
    RCLCPP_INFO(this->get_logger(), "RobotControllerNode initialized");
}

void RobotControllerNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
    ballanceController_->run();
    swingLegController_->run();
    jointController_->run();
    auto msg1 = jointController_->publishActuatorCmds();

    publisher_->publish(msg1);
    // RCLCPP_INFO(this->get_logger(), "Trigger received, running ballance controller. w: %.3f",
    // ballanceController_->w);
}

void RobotControllerNode::publish_commands()
{
    // auto msg = jointController_->publishActuatorCmds();

    // publisher_->publish(msg);
}

void RobotControllerNode::on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    // 将事件分发给各个子模块
    ballanceController_->set_parameters(event);
    swingLegController_->set_parameters(event);
    jointController_->set_parameters(event);
}

}  // namespace Galileo
