#include "robot_software/robot_controller/basic_controller/BallanceController.h"

#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include "robot_software/robot_utils/UtilFunc.h"
#include "sophus/so3.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#ifndef BALANCE_DEBUG
#define BALANCE_DEBUG 0  // 设置为1开启调试打印，设置为0关闭调试打印
#endif

#define BALANCE_PRINT(x)                 \
    do                                   \
    {                                    \
        if (BALANCE_DEBUG)               \
        {                                \
            std::cout << x << std::endl; \
        }                                \
    } while (0)
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
    auto Rz = Eigen::AngleAxisd(dataCenter_.read<robot_state::BaseState>()->eulerAngles(2), Eigen::Vector3d::UnitZ())
                  .toRotationMatrix();
    auto R = dataCenter_.read<robot_state::BaseState>()->rotationMatrix;
    Rz.setIdentity();
    // 1. 获取状态
    BALANCE_PRINT("Step 1: 获取当前状态和目标状态");
    const auto state_error = get_state_error();

    mat34 leg_pos_world = Rz.transpose() * dataCenter_.read<robot_state::LegState>()->legPosBaseInWorld;

    // leg_pos_world << 0.2, 0.2, -0.2, -0.2, -0.2, -0.2, 0.2, 0.2, -0.5, -0.5, -0.5, -0.5;

    const auto leg_phase = dataCenter_.read<robot_FSM::legState>()->legPhase;

    BALANCE_PRINT("状态误差: \n" << state_error.transpose());
    BALANCE_PRINT("支撑腿相位: " << leg_phase.transpose());

    const int active_legs = leg_phase.sum();
    BALANCE_PRINT("活动支撑腿数量: " << active_legs);
    if (active_legs == 0) return;

    // 2. 计算平衡力
    BALANCE_PRINT("\nStep 2: 计算平衡力");
    vec6 force = compute_balance_force(state_error);
    // force << 50, 0, 500, 0, 0, 0;
    BALANCE_PRINT("计算得到的平衡力: \n" << force.transpose());

    // 3. 构建QP约束矩阵
    BALANCE_PRINT("\nStep 3: 构建摩擦锥约束");
    const auto friction_cone = build_friction_cone_constraints();
    BALANCE_PRINT("摩擦锥约束矩阵: \n" << friction_cone);

    // 4. 构建QP基础矩阵
    BALANCE_PRINT("\nStep 4: 构建QP基础矩阵");
    QPMatrices qp_matrices;
    qp_matrices.A.setZero(6, active_legs * 3);
    qp_matrices.B = force;
    qp_matrices.C.setZero(active_legs * 6, active_legs * 3);
    qp_matrices.d.setZero(active_legs * 6);

    // 5. 填充约束矩阵
    BALANCE_PRINT("\nStep 5: 填充约束矩阵");
    int k = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (leg_phase(i))
        {
            qp_matrices.A.block(0, k * 3, 3, 3).setIdentity();
            qp_matrices.A.block(3, k * 3, 3, 3) = Sophus::SO3d::hat(leg_pos_world.col(i));
            qp_matrices.C.block(k * 6, k * 3, 6, 3) = friction_cone;
            qp_matrices.d(k * 6) = FORCE_MAX;
            qp_matrices.d(k * 6 + 1) = FORCE_MIN;
            k++;
        }
    }
    BALANCE_PRINT("约束矩阵A: \n" << qp_matrices.A);
    BALANCE_PRINT("约束矩阵C: \n" << qp_matrices.C);
    BALANCE_PRINT("约束向量d: \n" << qp_matrices.d.transpose());

    // 6. 构建权重矩阵
    BALANCE_PRINT("\nStep 6: 构建权重矩阵");
    const matxd W = 1e-3 * matxd::Identity(active_legs * 3, active_legs * 3);
    mat66 S = mat66::Identity();
    S.diagonal() << s_vec;
    BALANCE_PRINT("权重矩阵W: \n" << W << "\n权重矩阵S: \n" << S);

    // 7. 构建QP目标函数矩阵
    BALANCE_PRINT("\nStep 7: 构建QP目标函数矩阵");
    const matxd H = qp_matrices.A.transpose() * S * qp_matrices.A + W;
    const vecxd f = -qp_matrices.A.transpose() * S * qp_matrices.B;
    BALANCE_PRINT("H矩阵: \n" << H << "\nf向量: \n" << f.transpose());

    // 8. 求解QP问题
    BALANCE_PRINT("\nStep 8: 求解QP问题");
    Eigen::QuadProgDense qp;
    qp.problem(active_legs * 3, 0, active_legs * 6);
    // F_ext.resize(active_legs * 3);

    Eigen::Matrix<double, 0, Eigen::Dynamic> Aeq;
    Eigen::Matrix<double, 0, 1> Beq;
    Aeq.resize(0, active_legs * 3);
    if (qp.solve(H, f, Aeq, Beq, qp_matrices.C, qp_matrices.d))
    {
        F_ext = qp.result();
        BALANCE_PRINT("QP求解成功! 结果: \n" << F_ext.transpose());
    }
    else
    {
        BALANCE_PRINT("QP求解失败!");
    }

    robot_controller::BallanceController ballance;
    k = 0;
    for (int i = 0; i < 4; i++)
    {
        if (leg_phase(i))
        {
            ballance.f.col(i) = -Rz * F_ext.segment(k * 3, 3);
            k++;
        }
    }
    std::cout << "wrench: \n" << (qp_matrices.A * F_ext).transpose() << std::endl;
    std::cout << "pd: \n" << force.transpose() << std::endl;
    std::cout << "f: \n" << ballance.f << std::endl;
    std::cout << "F_ext: \n" << F_ext << std::endl;
    std::cout << "A: \n" << leg_pos_world << std::endl;
    std::cout << "Rz: \n" << Rz << std::endl;
    dataCenter_.write(ballance);
}

vec12 BallanceController::get_state_error() const
{
    vec12 error;
    auto R = dataCenter_.read<robot_state::BaseState>()->rotationMatrix;
    mat33 Rz = Eigen::AngleAxisd(dataCenter_.read<robot_state::BaseState>()->eulerAngles(2), Eigen::Vector3d::UnitZ())
                   .toRotationMatrix();
    // 将期望欧拉角转换为旋转矩阵
    mat33 R_des_mat;
    R_des_mat = dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetQuaternion.toRotationMatrix();

    // tf2::Matrix3x3 R_des_mat_tf(R_des_mat(0, 0),
    //                             R_des_mat(0, 1),
    //                             R_des_mat(0, 2),
    //                             R_des_mat(1, 0),
    //                             R_des_mat(1, 1),
    //                             R_des_mat(1, 2),
    //                             R_des_mat(2, 0),
    //                             R_des_mat(2, 1),
    //                             R_des_mat(2, 2));

    // tf2::Matrix3x3 R_mat_tf(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));

    // vec3 euler_angles_des;
    // R_des_mat_tf.getRPY(euler_angles_des(0), euler_angles_des(1), euler_angles_des(2));
    // vec3 euler_angles;
    // R_mat_tf.getRPY(euler_angles(0), euler_angles(1), euler_angles(2));

    // 计算旋转矩阵差值的向量化表示
    Eigen::Vector3d rotation_error = Sophus::SO3d::vee((R.transpose() * R_des_mat).log());
    // rotation_error = euler_angles_des - euler_angles;
    Rz.setIdentity();
    error.segment(0, 3) = (dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetPosition
                           - Rz.transpose() * dataCenter_.read<robot_state::BaseState>()->p_real);
    error.segment(3, 3) = rotation_error;
    error.segment(6, 3) = (dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetLinearVelocity
                           - Rz.transpose() * dataCenter_.read<robot_state::BaseState>()->v_real);
    error.segment(9, 3) = (dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetAngularVelocity
                           - Rz.transpose() * dataCenter_.read<robot_state::BaseState>()->angularVelocity);

    std::cout << "当前状态:" << std::endl;
    std::cout << "当前位置: " << (Rz.transpose() * dataCenter_.read<robot_state::BaseState>()->p_real).transpose()
              << std::endl;
    std::cout << "期望位置: "
              << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetPosition.transpose()
              << std::endl;
    std::cout << "当前姿态: " << dataCenter_.read<robot_state::BaseState>()->eulerAngles.transpose() << std::endl;
    std::cout << "期望姿态: "
              << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()
                     ->targetQuaternion.toRotationMatrix()
                     .eulerAngles(0, 1, 2)
                     .transpose()
              << std::endl;
    std::cout << "当前线速度: " << (Rz.transpose() * dataCenter_.read<robot_state::BaseState>()->v_real).transpose()
              << std::endl;
    std::cout << "期望线速度: "
              << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetLinearVelocity.transpose()
              << std::endl;
    std::cout << "当前角速度: "
              << (Rz.transpose() * dataCenter_.read<robot_state::BaseState>()->angularVelocity).transpose()
              << std::endl;
    std::cout << "期望角速度: "
              << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetAngularVelocity.transpose()
              << std::endl;

    std::cout << "rotation_error: \n" << rotation_error << std::endl;

    return error;
}

vec6 BallanceController::compute_balance_force(const vec12& state_error) const
{
    vec6 f = kp.cwiseProduct(state_error.head(6)) + kd.cwiseProduct(state_error.tail(6));
    f(2) += dataCenter_.read<robot_constants>()->mass * 9.8;
    f(0) += 400 * (0.0 - dataCenter_.read<robot_state::BaseState>()->positionRelative(0));
    return f;
}

Eigen::Matrix<double, 6, 3> BallanceController::build_friction_cone_constraints() const
{
    constexpr double ux = 0.7;
    constexpr double uy = 0.7;

    Eigen::Matrix<double, 6, 3> friction_cone;
    friction_cone << 0, 0, 1, 0, 0, -1, 1, 0, -ux * 0.7, 0, 1, -uy * 0.7, -1, 0, -ux * 0.7, 0, -1, -uy * 0.7;

    return friction_cone;
}

void BallanceController::declare_and_get_parameters(rclcpp::Node* node)
{
    // 声明参数
    node->declare_parameter("ballance_controller.kp", std::vector<double>{6, 0});
    node->declare_parameter("ballance_controller.kd", std::vector<double>{6, 0});
    node->declare_parameter("ballance_controller.s", std::vector<double>{6, 0});
    node->declare_parameter<double>("ballance_controller.w", 0);
    node->declare_parameter<double>("ballance_controller.force_max", 0);
    node->declare_parameter<double>("ballance_controller.force_min", 0);

    kp = stdVectorToEigen<vec6>(node->get_parameter("ballance_controller.kp").as_double_array());
    kd = stdVectorToEigen<vec6>(node->get_parameter("ballance_controller.kd").as_double_array());
    s_vec = stdVectorToEigen<vec6>(node->get_parameter("ballance_controller.s").as_double_array());
    w = node->get_parameter("ballance_controller.w").as_double();
    FORCE_MAX = node->get_parameter("ballance_controller.force_max").as_double();
    FORCE_MIN = node->get_parameter("ballance_controller.force_min").as_double();
}

void BallanceController::set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for (const auto& changed_parameter : event->changed_parameters)
    {
        const auto& name = changed_parameter.name;
        if (name == "ballance_controller.kp")
        {
            kp = stdVectorToEigen<vec6>(changed_parameter.value.double_array_value);
        }
        else if (name == "ballance_controller.kd")
        {
            kd = stdVectorToEigen<vec6>(changed_parameter.value.double_array_value);
        }
        else if (name == "ballance_controller.s")
        {
            s_vec = stdVectorToEigen<vec6>(changed_parameter.value.double_array_value);
        }
        else if (name == "ballance_controller.w")
        {
            w = changed_parameter.value.double_value;
        }
        else if (name == "ballance_controller.force_max")
        {
            FORCE_MAX = changed_parameter.value.double_value;
        }
        else if (name == "ballance_controller.force_min")
        {
            FORCE_MIN = changed_parameter.value.double_value;
        }
    }
}
}  // namespace Galileo
