#include "robot_software/robot_controller/BallanceController.h"

#include "robot_software/robot_utils/UtilFunc.h"
#include "sophus/so3.hpp"

namespace Galileo
{

BallanceController::BallanceController()
    : dataCenter_(DataCenter::getInstance())
{
    // F_ext.resize(12);  // 预分配内存
}

BallanceController::~BallanceController() = default;

void BallanceController::run()
{
    // 1. 获取状态
    const auto [state, state_des] = get_current_and_target_state();
    const mat34 leg_pos_world = dataCenter_.read<robot_state::LegState>()->legPosBaseInWorld;
    const auto leg_phase = dataCenter_.read<robot_FSM::legState>()->legPhase;

    const int active_legs = leg_phase.sum();
    if (active_legs == 0) return;

    // 2. 计算平衡力
    const vec6 force = compute_balance_force(state, state_des);

    // 3. 构建QP约束矩阵
    const auto friction_cone = build_friction_cone_constraints();

    // 4. 构建QP基础矩阵
    QPMatrices qp_matrices;
    qp_matrices.A.setZero(6, active_legs * 3);
    qp_matrices.B = force;
    qp_matrices.C.setZero(active_legs * 6, active_legs * 3);
    qp_matrices.d.setZero(active_legs * 6);

    // 5. 填充约束矩阵
    int k = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (leg_phase(i))
        {
            qp_matrices.A.block(3, k * 3, 3, 3) = Sophus::SO3d::hat(leg_pos_world.col(i));
            qp_matrices.C.block(k * 6, k * 3, 6, 3) = friction_cone;
            qp_matrices.d(k * 6) = FORCE_MAX;
            qp_matrices.d(k * 6 + 1) = FORCE_MIN;
            k++;
        }
    }

    // 6. 构建权重矩阵
    const matxd W = 1e-3 * matxd::Identity(active_legs * 3, active_legs * 3);
    mat66 S;
    S.diagonal() << s_vec;

    // 7. 构建QP目标函数矩阵
    const matxd H = qp_matrices.A.transpose() * S * qp_matrices.A + W;
    const vecxd f = -qp_matrices.A.transpose() * S * qp_matrices.B;

    // 8. 求解QP问题
    Eigen::QuadProgDense qp;
    qp.problem(active_legs * 3, 0, active_legs * 6);

    if (qp.solve(H, f, matxd(), vecxd(), qp_matrices.C, qp_matrices.d))
    {
        F_ext = qp.result();
    }
}

std::tuple<vec12, vec12> BallanceController::get_current_and_target_state() const
{
    vec12 state, state_des;

    state << dataCenter_.read<robot_state::BaseState>()->position,
        dataCenter_.read<robot_state::BaseState>()->eulerAngles,
        dataCenter_.read<robot_state::BaseState>()->linearVelocity,
        dataCenter_.read<robot_state::BaseState>()->angularVelocity;

    state_des << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetPosition,
        dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetEulerAngles,
        dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetLinearVelocity,
        dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetAngularVelocity;

    return {state, state_des};
}

vec6 BallanceController::compute_balance_force(const vec12& state, const vec12& state_des) const
{
    return kp.cwiseProduct(state_des.head(6) - state.head(6)) + kd.cwiseProduct(state_des.tail(6) - state.tail(6));
}

Eigen::Matrix<double, 6, 3> BallanceController::build_friction_cone_constraints() const
{
    constexpr double ux = 0.75;
    constexpr double uy = 0.75;

    Eigen::Matrix<double, 6, 3> friction_cone;
    friction_cone << 0, 0, 1, 0, 0, -1, 1, 0, -ux, 0, 1, -uy, -1, 0, -ux, 0, -1, -uy;

    return friction_cone;
}

void BallanceController::declare_and_get_parameters(rclcpp::Node* node)
{
    // 声明参数
    node->declare_parameter("kp", std::vector<double>{0, 0, 0, 0, 0, 0});
    node->declare_parameter("kd", std::vector<double>{0, 0, 0, 0, 0, 0});
    node->declare_parameter("s", std::vector<double>{0, 0, 0, 0, 0, 0});
    node->declare_parameter<double>("w", 0);
    node->declare_parameter<double>("force_max", 0);
    node->declare_parameter<double>("force_min", 0);

    kp = stdVectorToEigen<vec6>(node->get_parameter("kp").as_double_array());
    kd = stdVectorToEigen<vec6>(node->get_parameter("kd").as_double_array());
    s_vec = stdVectorToEigen<vec6>(node->get_parameter("s").as_double_array());
    w = node->get_parameter("w").as_double();
    FORCE_MAX = node->get_parameter("force_max").as_double();
    FORCE_MIN = node->get_parameter("force_min").as_double();
}

void BallanceController::set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for (const auto& changed_parameter : event->changed_parameters)
    {
        const auto& name = changed_parameter.name;
        if (name == "kp")
        {
            kp = stdVectorToEigen<vec6>(changed_parameter.value.double_array_value);
        }
        else if (name == "kd")
        {
            kd = stdVectorToEigen<vec6>(changed_parameter.value.double_array_value);
        }
        else if (name == "s")
        {
            s_vec = stdVectorToEigen<vec6>(changed_parameter.value.double_array_value);
        }
        else if (name == "w")
        {
            w = changed_parameter.value.double_value;
        }
        else if (name == "force_max")
        {
            FORCE_MAX = changed_parameter.value.double_value;
        }
        else if (name == "force_min")
        {
            FORCE_MIN = changed_parameter.value.double_value;
        }
    }
}
}  // namespace Galileo
