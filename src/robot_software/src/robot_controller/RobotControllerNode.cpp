#include "robot_software/robot_controller/RobotControllerNode.h"

using namespace std::chrono_literals;

namespace Galileo
{
RobotControllerNode::RobotControllerNode()
    : Node("robot_controller")
{
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

    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published actuator commands %.3f", msg.torque[0]);
}
}  // namespace Galileo
