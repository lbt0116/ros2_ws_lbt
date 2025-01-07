#include "robot_software/robot_controller/basic_controller/SwingLegController.h"

#include "robot_software/robot_utils/UtilFunc.h"
namespace Galileo
{
SwingLegController::SwingLegController()
    : dataCenter_(DataCenter::getInstance())
{
}

SwingLegController::~SwingLegController()
{
}

void SwingLegController::run()
{
    auto p = dataCenter_.read<robot_state::LegState>()->legPosHipInWorld;  // with yaw
    auto v = dataCenter_.read<robot_state::LegState>()->legVeloInWorld;    // with yaw

    auto p_des = dataCenter_.read<robot_target_trajectory::TargetLegTrajectory>()->targetLegPosition;  // w/o yaw
    auto v_des = dataCenter_.read<robot_target_trajectory::TargetLegTrajectory>()->targetLegVelocity;  // w/o yaw
    auto Rz = Eigen::AngleAxisd(dataCenter_.read<robot_state::BaseState>()->eulerAngles(2), Eigen::Vector3d::UnitZ())
                  .toRotationMatrix();

    auto R = dataCenter_.read<robot_state::BaseState>()->rotationMatrix;
    // p_des << 0.1, 0.0, 0, 0, 0.2, 0, 0, 0, -0.5, -0.5, -0.5, -0.5;

    mat34 f = kp.cwiseProduct(Rz * p_des - p) + kd.cwiseProduct(Rz * v_des - v);  // cuole?
    // mat34 f = kp.cwiseProduct(p_des - p) + kd.cwiseProduct(v_des - v);  // cuole?
    robot_controller::SwingLegController sw;
    // f.row(0).setConstant(0);
    // f.row(1).setConstant(10);
    // f.row(2).setConstant(0);
    sw.f =  f;
    dataCenter_.write(sw);
    // std::cout << "p: " << p << std::endl;
    // std::cout << "p_des: " << p_des << std::endl;
    // std::cout << "Rz.transpose() * v: " << Rz.transpose() * v << std::endl;
    // std::cout << "v: " << v << std::endl;
    // std::cout << "v_des: " << v_des << std::endl;
    // std::cout << "f: " << f << std::endl;
    // std::cout << "swf: " << sw.f << std::endl;
    // std::cout << "Rz: " << Rz << std::endl;
    // std::cout << "yaw: " << dataCenter_.read<robot_state::BaseState>()->eulerAngles(2) << std::endl;
    // std::cout << "kp: " << kp.cwiseProduct(p_des - p) << std::endl;
    // std::cout << "kd: " << kd.cwiseProduct(v_des - Rz.transpose() * v) << std::endl;
}

void SwingLegController::declare_and_get_parameters(rclcpp::Node* node)
{
    // 声明参数
    node->declare_parameter("leg_controller.kp", std::vector<double>{12, 0});
    node->declare_parameter("leg_controller.kd", std::vector<double>{12, 0});

    kp = stdVectorToEigen<mat34>(node->get_parameter("leg_controller.kp").as_double_array());
    kd = stdVectorToEigen<mat34>(node->get_parameter("leg_controller.kd").as_double_array());
}

void SwingLegController::set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for (const auto& changed_parameter : event->changed_parameters)
    {
        const auto& name = changed_parameter.name;
        if (name == "leg_controller.kp")
        {
            kp = stdVectorToEigen<mat34>(changed_parameter.value.double_array_value);
        }
        else if (name == "leg_controller.kd")
        {
            kd = stdVectorToEigen<mat34>(changed_parameter.value.double_array_value);
        }
    }
}
}  // namespace Galileo