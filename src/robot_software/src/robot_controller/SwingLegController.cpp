#include "robot_software/robot_controller/SwingLegController.h"

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
    auto p = dataCenter_.read<robot_state::LegState>()->legPosHipInWorld;
    auto v = dataCenter_.read<robot_state::LegState>()->legVeloInWorld;

    auto p_des = dataCenter_.read<robot_target_trajectory::TargetLegTrajectory>()->targetLegPosition;
    auto v_des = dataCenter_.read<robot_target_trajectory::TargetLegTrajectory>()->targetLegVelocity;
    p_des.col(0) << 0, 0, -0.5;
    p_des.col(1) << 0, 0, -0.5;
    p_des.col(2) << 0, 0, -0.5;
    p_des.col(3) << 0, 0, -0.5;
    v_des << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    mat34 f = kp.cwiseProduct(p_des - p) + kd.cwiseProduct(v_des - v);
    robot_controller::SwingLegController sw;
    sw.f = f;
    dataCenter_.write(sw);

    // std::cout<<"p      "<<p<<std::endl;
    // std::cout<<"p_des  "<<p_des<<std::endl;
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