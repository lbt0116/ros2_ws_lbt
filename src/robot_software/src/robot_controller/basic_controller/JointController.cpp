#include "robot_software/robot_controller/basic_controller/JointController.h"

#include "robot_software/robot_utils/UtilFunc.h"

namespace Galileo
{
JointController::JointController()
    : dataCenter_(DataCenter::getInstance())
{
}

JointController::~JointController() = default;

void JointController::run()
{
    // 获取关节位置、速度、力矩
    auto q = dataCenter_.read<robot_state::JointState>()->jointPosition;
    auto dq = dataCenter_.read<robot_state::JointState>()->jointVelocity;

    auto q_des = dataCenter_.read<robot_target_trajectory::TargetJointTrajectory>()->targetJointPosition;
    auto dq_des = dataCenter_.read<robot_target_trajectory::TargetJointTrajectory>()->targetJointVelocity;

    mat34 tau = kp.cwiseProduct(q_des - q) + kd.cwiseProduct(dq_des - dq);

    compose_leg_force();
    compute_joint_torque();

    // std::cout << "tau: " << jointController_.tau << std::endl;
    // std::cout << "joint pd tau: " << tau << std::endl;
    // std::cout << "composedLegForce: " << jointController_.composedLegForce << std::endl;
}
void JointController::compose_leg_force()
{
    auto f_st = dataCenter_.read<robot_controller::BallanceController>()->f;
    auto f_sw = dataCenter_.read<robot_controller::SwingLegController>()->f;
    auto phase = dataCenter_.read<robot_FSM::legState>()->legPhase;

    for (int i = 0; i < 4; i++)
        if (phase(i))
        {
            jointController_.composedLegForce.col(i) = f_st.col(i);
        }
        else
        {
            jointController_.composedLegForce.col(i) = f_sw.col(i);
        }
}

void JointController::compute_joint_torque()
{
    auto jacobian = dataCenter_.read<robot_state::LegState>()->legJacobian;
    Eigen::VectorXd t = Eigen::Matrix<double, 18, 1>::Zero();
    for (int i = 0; i < 4; i++)
    {
        t += jacobian[i].topRows(3).transpose() * jointController_.composedLegForce.col(i);
        // std::cout << "t" << i << "  " << jacobian[i].topRows(3).transpose() *
        // jointController_.composedLegForce.col(i)
        //           << std::endl;
    }
    jointController_.tau = Eigen::Map<Eigen::Matrix<double, 3, 4>>(t.tail(12).data());
    dataCenter_.write(jointController_);
}

custom_msgs::msg::ActuatorCmds JointController::publishActuatorCmds()
{
    auto msg = custom_msgs::msg::ActuatorCmds();  // 初始化关节名称
    auto joint_names_ = {"Abd1Joint",
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
    auto ctrl = dataCenter_.read<robot_user_cmd::UserCmd>()->ctrlType;
    // 设置控制命令
    for (size_t i = 0; i < num_joints; i++)
    {
        // 这里可以根据实际控制需求设置不同的值
        if (ctrl == 1)
        {
            msg.torque[i] = jointController_.tau(i);
            // msg.torque[i] = 0;
            msg.torque_limit[i] = 100.0;
            msg.pos_limit[i] = 3.14;  // pi弧度
            msg.pos[i] = 0.0;
            msg.vel[i] = 0.0;
            msg.kp[i] = 0.0;  // PD控制增益
            msg.kd[i] = 0.0;
        }
        else
        {
            msg.torque[i] = 0;
            // msg.torque[i] = 0;
            msg.torque_limit[i] = 100.0;
            msg.pos_limit[i] = 3.14;  // pi弧度
            msg.pos[i] = 0.0;
            msg.vel[i] = 0.0;
            msg.kp[i] = 300.0;  // PD控制增益
            msg.kd[i] = 20.0;
        }
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

    return msg;
}

void JointController::declare_and_get_parameters(rclcpp::Node* node)
{
    // 声明参数
    node->declare_parameter("joint_controller.kp", std::vector<double>{12, 1});
    node->declare_parameter("joint_controller.kd", std::vector<double>{12, 1});

    kp = stdVectorToEigen<mat34>(node->get_parameter("joint_controller.kp").as_double_array());
    kd = stdVectorToEigen<mat34>(node->get_parameter("joint_controller.kd").as_double_array());
}

void JointController::set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for (const auto& changed_parameter : event->changed_parameters)
    {
        const auto& name = changed_parameter.name;
        if (name == "joint_controller.kp")
        {
            kp = stdVectorToEigen<mat34>(changed_parameter.value.double_array_value);
        }
        else if (name == "joint_controller.kd")
        {
            kd = stdVectorToEigen<mat34>(changed_parameter.value.double_array_value);
        }
    }
}

}  // namespace Galileo
