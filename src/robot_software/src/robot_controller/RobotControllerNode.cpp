#include "robot_software/robot_controller/RobotControllerNode.h"

using namespace std::chrono_literals;

namespace Galileo
{
RobotControllerNode::RobotControllerNode()
    : Node("robot_controller", rclcpp::NodeOptions().use_intra_process_comms(true)),
      dataCenter(DataCenter::getInstance()),
      ballanceController_(std::make_unique<BallanceController>())
{
    // 声明参数
    ballanceController_->declare_and_get_parameters(this);
    //
    parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
        "/parameter_events",
        10,
        std::bind(&BallanceController::set_parameters, ballanceController_.get(), std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DynamicParameterNode initialized and listening for parameter events.");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    publisher_ = this->create_publisher<custom_msgs::msg::ActuatorCmds>("actuators_cmds", qos);
    timer_ = this->create_wall_timer(1ms, std::bind(&RobotControllerNode::publish_commands, this));

    // 初始化关节名称
    joint_names_ = {"Abd1Joint",
                    "Hip1Joint",
                    "Knee1Joint",
                    "Abd2Joint",
                    "Hip2Joint",
                    "Knee2Joint",
                    "Abd3Joint",
                    "Hip3Joint",
                    "Knee3Joint",
                    "Abd4Joint",
                    "Hip4Joint",
                    "Knee4Joint"};

    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&RobotControllerNode::trigger_callback, this, std::placeholders::_1));  // 订阅触发信号
}

void RobotControllerNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
{
    ballanceController_->run();
    RCLCPP_INFO(this->get_logger(), "Trigger received, running ballance controller. w: %.3f", ballanceController_->w);
}

void RobotControllerNode::publish_commands()
{
    auto msg = custom_msgs::msg::ActuatorCmds();
    msg.actuators_name.assign(joint_names_.begin(), joint_names_.end());

    // 调整所有数组大小
    const size_t num_joints = joint_names_.size();
    msg.torque.resize(num_joints);
    msg.torque_limit.resize(num_joints);
    msg.pos.resize(num_joints);
    msg.vel.resize(num_joints);
    msg.kp.resize(num_joints);
    msg.kd.resize(num_joints);
    msg.pos_limit.resize(num_joints);

    // 设置控制命令
    for (size_t i = 0; i < num_joints; i++)
    {
        // 这里可以根据实际控制需求设置不同的值
        msg.torque[i] = 0.0;
        msg.torque_limit[i] = 100.0;
        msg.pos_limit[i] = 3.14;  // pi弧度
        msg.pos[i] = 0.0;
        msg.vel[i] = 0.0;
        msg.kp[i] = 100.0;  // PD控制增益
        msg.kd[i] = 10.0;
    }
    msg.pos[0] = 0.0;
    msg.pos[3] = 0.0;
    msg.pos[6] = 0.0;
    msg.pos[9] = 0.0;

    msg.pos[1] = -3.14 / 4;
    msg.pos[4] = -3.14 / 4;
    msg.pos[7] = -3.14 / 4;
    msg.pos[10] = -3.14 / 4;

    msg.pos[2] = 3.14 / 2;
    msg.pos[5] = 3.14 / 2;
    msg.pos[8] = 3.14 / 2;
    msg.pos[11] = 3.14 / 2;

    publisher_->publish(msg);
    // RCLCPP_INFO(this->get_logger(), "Published actuator commands %.3f", msg.torque[0]);
}
}  // namespace Galileo
