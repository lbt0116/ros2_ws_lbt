#pragma once

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/DataTypes.hpp"

namespace Galileo
{

class JointController
{
public:
    JointController();
    ~JointController();

    void run();

    void declare_and_get_parameters(rclcpp::Node* node);

    void set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    custom_msgs::msg::ActuatorCmds publishActuatorCmds();

private:
    DataCenter& dataCenter_;
    mat34 kp;
    mat34 kd;

    void compose_leg_force();
    void compute_joint_torque();

    robot_controller::JointController jointController_;
};
}  // namespace Galileo