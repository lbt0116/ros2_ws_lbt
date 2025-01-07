#define DEBUG_NMPC 1  // 设置为1开启调试输出,0关闭
#define NUM_LEGS 4    // 机器人腿的数量

#if DEBUG_NMPC
#define DEBUG_PRINT(x) std::cout << x << std::endl
#define DEBUG_MATRIX(name, mat) std::cout << #name ": \n" << mat << std::endl
#else
#define DEBUG_PRINT(x)
#define DEBUG_MATRIX(name, mat)
#endif

#include "robot_software/robot_controller/nmpc_controller/GeometricNmpc.h"

#include <Eigen/Core>
#include <chrono>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;
namespace Galileo
{
namespace
{
// 辅助函数：构建分块对角矩阵
template <typename Derived>
void buildBlockDiagonalMatrix(MatrixXd& output,
                              const MatrixBase<Derived>& weights,
                              int block_size,
                              int num_blocks,
                              int block_repeat = 1)
{
    const auto diagonal = weights.asDiagonal();
    const int weight_size = weights.size();

    for (int i = 0; i < num_blocks; ++i)
    {
        for (int j = 0; j < block_repeat; ++j)
        {
            const int start_idx = i * block_size + j * weight_size;
            if (start_idx + weight_size <= output.rows())
            {
                output.block(start_idx, start_idx, weight_size, weight_size) = diagonal;
            }
        }
    }
}

// 辅助函数：从参数获取向量
VectorXd getWeightVector(const std::vector<double>& array, int expected_size)
{
    if (array.empty() || array.size() != expected_size)
    {
        return VectorXd::Zero(expected_size);
    }
    return Map<const VectorXd>(array.data(), array.size());
}

Matrix3d skew(const Vector3d& vec)
{
    Matrix3d m;
    m << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return m;
}

Vector3d invskew(const Matrix3d& Mat)
{
    Vector3d v;
    v(0) = Mat(2, 1);
    v(1) = Mat(0, 2);
    v(2) = Mat(1, 0);
    return v;
}

Matrix<double, 6, 6> Adjoint(const Matrix<double, 4, 4>& M)
{
    Matrix<double, 6, 6> Ad;
    Matrix<double, 3, 3> R = M.block(0, 0, 3, 3);
    Vector3d p = M.block(0, 3, 3, 1);

    Ad.setZero();
    Ad.block(0, 0, 3, 3) = R;
    Ad.block(3, 0, 3, 3) = skew(p) * R;
    Ad.block(3, 3, 3, 3) = R;
    return Ad;
}

Matrix<double, 6, 6> adjoint(const Matrix<double, 6, 1>& Twist)
{
    Matrix<double, 6, 6> ad;
    Vector3d w = Twist.block(0, 0, 3, 1);
    Vector3d v = Twist.block(3, 0, 3, 1);

    ad.setZero();
    ad.block(0, 0, 3, 3) = skew(w);
    ad.block(3, 0, 3, 3) = skew(v);
    ad.block(3, 3, 3, 3) = skew(w);
    return ad;
}

}  // namespace

GeometricNmpc::GeometricNmpc()
    : dataCenter_(DataCenter::getInstance())
{
}

void GeometricNmpc::declare_and_get_parameters(rclcpp::Node* node)
{
    // 声明参数
    const std::vector<std::pair<std::string, double>> scalar_params = {
        {"dt_MPC", 0.0}, {"u", 0.0}, {"fmax", 0.0}, {"fmin", 0.0}};
    for (const auto& [name, default_value] : scalar_params)
    {
        node->declare_parameter<double>("geometric_nmpc." + name, default_value);
    }
    node->declare_parameter<int>("geometric_nmpc.n_steps", 0);

    const std::vector<std::string> vector_params = {"vec_Q", "vec_R", "vec_P"};
    for (const auto& name : vector_params)
    {
        node->declare_parameter<std::vector<double>>("geometric_nmpc." + name, {});
    }

    // 获取参数值
    dt_MPC = node->get_parameter("geometric_nmpc.dt_MPC").as_double();
    n_steps = node->get_parameter("geometric_nmpc.n_steps").as_int();
    u = node->get_parameter("geometric_nmpc.u").as_double();
    fmax = node->get_parameter("geometric_nmpc.fmax").as_double();
    fmin = node->get_parameter("geometric_nmpc.fmin").as_double();

    // 获取权重向量
    const auto q_vec = node->get_parameter("geometric_nmpc.vec_Q").as_double_array();
    const auto r_vec = node->get_parameter("geometric_nmpc.vec_R").as_double_array();
    const auto p_vec = node->get_parameter("geometric_nmpc.vec_P").as_double_array();

    if (q_vec.empty() || r_vec.empty() || p_vec.empty())
    {
        DEBUG_PRINT("Warning: One or more weight vectors are empty!");
        return;
    }

    // 初始化维度
    const int state_dim = q_vec.size();
    const int single_leg_control_dim = r_vec.size();

    // 初始化权重矩阵
    weight_Q.resize(state_dim, state_dim);
    weight_R.resize(single_leg_control_dim * NUM_LEGS, single_leg_control_dim * NUM_LEGS);
    weight_P.resize(state_dim, state_dim);
    weight_Q.setZero();
    weight_R.setZero();
    weight_P.setZero();

    // 构建权重矩阵
    const VectorXd q = getWeightVector(q_vec, state_dim);
    const VectorXd r = getWeightVector(r_vec, single_leg_control_dim);
    const VectorXd p = getWeightVector(p_vec, state_dim);

    weight_Q = q.asDiagonal();
    weight_P = p.asDiagonal();
    buildBlockDiagonalMatrix(weight_R, r, single_leg_control_dim, NUM_LEGS);

    // 获取物理参数
    mass = dataCenter_.read<robot_constants>()->mass;
    gravity = dataCenter_.read<robot_constants>()->gravity;
    Inertia.setZero();
    Inertia.diagonal() = dataCenter_.read<robot_constants>()->inertiaVector;
    Jb = Eigen::MatrixXd::Zero(6, 6);
    Jb.topLeftCorner(3, 3) = Inertia;
    Jb.bottomRightCorner(3, 3) = mass * Eigen::Matrix3d::Identity();
    Jb_inv = Jb.inverse();

    if (DEBUG_NMPC)
    {
        DEBUG_PRINT("Dimensions: state=" << state_dim << ", control=" << single_leg_control_dim
                                         << ", steps=" << n_steps);
        DEBUG_PRINT("Q: " << weight_Q.rows() << "x" << weight_Q.cols());
        DEBUG_MATRIX("Q", weight_Q);
        DEBUG_PRINT("R: " << weight_R.rows() << "x" << weight_R.cols());
        DEBUG_MATRIX("R", weight_R);
    }
}

void GeometricNmpc::set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
    for (const auto& param : event->changed_parameters)
    {
        const auto& name = param.name;
        const auto& value = param.value;

        if (name == "geometric_nmpc.dt_MPC")
            dt_MPC = value.double_value;
        else if (name == "geometric_nmpc.u")
            u = value.double_value;
        else if (name == "geometric_nmpc.fmax")
            fmax = value.double_value;
        else if (name == "geometric_nmpc.fmin")
            fmin = value.double_value;
        else if (name == "geometric_nmpc.vec_Q" && !value.double_array_value.empty())
        {
            const auto& array = value.double_array_value;
            if (array.size() == weight_Q.rows())
            {
                weight_Q = getWeightVector(array, array.size()).asDiagonal();
            }
        }
        else if (name == "geometric_nmpc.vec_R" && !value.double_array_value.empty())
        {
            const auto& array = value.double_array_value;
            const int control_dim = array.size() * NUM_LEGS;
            if (control_dim == weight_R.rows())
            {
                buildBlockDiagonalMatrix(weight_R, getWeightVector(array, array.size()), array.size(), NUM_LEGS);
            }
        }
    }
}

void GeometricNmpc::update_dimensions()
{
    num_contact = contactLegPhase.sum();
    control_dim = num_contact * single_leg_dim;
    single_step_dim = state_dim + control_dim;
    total_dim = single_step_dim * n_steps;
    constraints_per_step = num_contact * single_leg_dim;
    total_constraints = constraints_per_step * n_steps;
}

void GeometricNmpc::set_input()
{
    // p q j v w jv
    p << dataCenter_.read<robot_state::BaseState>()->position;
    R << dataCenter_.read<robot_state::BaseState>()->rotationMatrix;
    v << dataCenter_.read<robot_state::BaseState>()->linearVelocity;
    w << dataCenter_.read<robot_state::BaseState>()->angularVelocity;

    p_d << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetPosition;
    R_d << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetQuaternion.toRotationMatrix();
    v_d << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetLinearVelocity;
    w_d << dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetAngularVelocity;

    contactLegPos = dataCenter_.read<robot_state::LegState>()->legPosBaseInWorld;
    contactLegPhase = dataCenter_.read<robot_FSM::legState>()->legPhase;

    // 初始化力为零（如果是第一次运行）
    if (forceResult.isZero())
    {
        forceResult.setZero();
        // 设置一个初始的垂直力（重力平均分配）
        int num_contact = contactLegPhase.sum();
        if (num_contact > 0)
        {
            double init_force = -mass * gravity[2] / num_contact;
            int contact_idx = 0;
            for (int leg = 0; leg < NUM_LEGS; ++leg)
            {
                if (contactLegPhase[leg])
                {
                    forceResult(2, leg) = init_force;  // 设置z方向的力
                }
            }
        }
    }

    SE3_d << R_d, p_d, 0, 0, 0, 1;
    SE3 << R, p, 0, 0, 0, 1;

    se3_d << w_d, v_d;
    se3 << w, v;

    // 更新维度
    update_dimensions();
}

void GeometricNmpc::set_mat_A()
{
    // 计算基本的A矩阵块
    mat_A.resize(12, 12);
    mat_A.setZero();

    // 上半部分
    mat_A.block<6, 6>(0, 0) = Matrix<double, 6, 6>::Identity() - adjoint(se3) * dt_MPC;
    mat_A.block<6, 6>(0, 6) = Matrix<double, 6, 6>::Identity();

    // 计算G矩阵
    Matrix<double, 6, 6> G;
    G.setZero();
    Vector3d Iw = Inertia * w;
    G.block(0, 0, 3, 3) = -skew(Iw);
    G.block(0, 3, 3, 3) = -mass * skew(v);
    G.block(3, 0, 3, 3) = -mass * skew(v);

    // 计算E矩阵
    Matrix<double, 6, 6> E;
    E.setZero();
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        if (contactLegPhase[i])
        {
            Vector3d f = forceResult.col(i);  // 使用上一次的计算结果
            E.block(3, 0, 3, 3) += skew(f);
        }
    }

    // 下半部分
    mat_A.block<6, 6>(6, 0) = E;
    mat_A.block<6, 6>(6, 6) = Matrix<double, 6, 6>::Identity() - Jb_inv * (G + adjoint(se3).transpose() * Jb) * dt_MPC;
}

void GeometricNmpc::set_mat_B()
{
    int num_contact = contactLegPhase.sum();
    const int control_dim = num_contact * 3;

    // 初始化B矩阵
    mat_B.resize(12, control_dim);
    mat_B.setZero();

    // 计算接触雅可比矩阵T
    Matrix<double, 6, Eigen::Dynamic> T;
    T.resize(6, control_dim);
    T.setZero();

    int contact_idx = 0;
    for (int i = 0; i < NUM_LEGS; ++i)
    {
        if (contactLegPhase[i])
        {
            Vector3d r = contactLegPos.col(i);
            T.block<3, 3>(0, contact_idx * 3) = skew(r);
            T.block<3, 3>(3, contact_idx * 3) = Matrix3d::Identity();
            contact_idx++;
        }
    }

    // 设置B矩阵
    mat_B.topRows(6).setZero();
    mat_B.bottomRows(6) = Jb_inv * T * dt_MPC;
}

void GeometricNmpc::set_mat_C()
{
    // 基本约束矩阵C1（对应单个接触点）
    Matrix<double, 3, 3> C1 = Matrix3d::Identity();

    // 初始化约束矩阵
    mat_C.resize(total_constraints, total_dim);
    mat_C.setZero();

    // 设置约束矩阵
    for (int step = 0; step < n_steps; ++step)
    {
        int contact_idx = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            if (contactLegPhase[leg])
            {
                // 计算当前块的起始位置
                const int col_start = step * control_dim + contact_idx * single_leg_dim;
                const int row_start = step * constraints_per_step + contact_idx * single_leg_dim;

                // 设置单腿约束矩阵
                mat_C.block<3, 3>(row_start, col_start) = C1;

                contact_idx++;
            }
        }
    }
}

void GeometricNmpc::set_vec_l()
{
    // 初始化约束向量
    vec_l.resize(total_constraints);
    vec_l.setZero();

    // 对每个时间步设置约束
    for (int step = 0; step < n_steps; ++step)
    {
        int contact_idx = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            if (contactLegPhase[leg])
            {
                const int start_idx = step * constraints_per_step + contact_idx * single_leg_dim;
                Vector3d f = forceResult.col(leg);  // 使用上一次的计算结果

                // 设置约束下界
                vec_l[start_idx] = -u * f[2];        // fx下界：-μfz
                vec_l[start_idx + 1] = -u * f[2];    // fy下界：-μfz
                vec_l[start_idx + 2] = fmin - f[2];  // fz下界：fmin - fz

                contact_idx++;
            }
        }
    }
}

void GeometricNmpc::set_vec_u()
{
    // 初始化约束向量
    vec_u.resize(total_constraints);
    vec_u.setZero();

    // 对每个时间步设置约束
    for (int step = 0; step < n_steps; ++step)
    {
        int contact_idx = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            if (contactLegPhase[leg])
            {
                const int start_idx = step * constraints_per_step + contact_idx * single_leg_dim;
                Vector3d f = forceResult.col(leg);  // 使用上一次的计算结果

                // 设置约束上界
                vec_u[start_idx] = u * f[2];         // fx上界：μfz
                vec_u[start_idx + 1] = u * f[2];     // fy上界：μfz
                vec_u[start_idx + 2] = fmax - f[2];  // fz上界：fmax - fz

                contact_idx++;
            }
        }
    }
}

void GeometricNmpc::set_mat_H()
{
    // 初始化H矩阵
    mat_H.resize(total_dim, total_dim);
    mat_H.setZero();

    // 计算期望力f_d（重力平均分配）
    Vector3d f_d;
    f_d.setZero();
    f_d[2] = -mass * gravity[2] / num_contact;

    // 对每个预测步设置H矩阵块
    for (int step = 0; step < n_steps; ++step)
    {
        const int block_start = step * single_step_dim;

        // 设置Q块（中间步骤使用Q，最后一步使用P）
        if (step < n_steps - 1)
        {
            mat_H.block(block_start, block_start, state_dim, state_dim) = weight_Q;
        }
        else
        {
            mat_H.block(block_start, block_start, state_dim, state_dim) = weight_P;
        }

        // 设置R块（只使用接触腿的部分）
        int contact_idx = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            if (contactLegPhase[leg])
            {
                const int force_start = block_start + state_dim + contact_idx * single_leg_dim;
                mat_H.block(force_start, force_start, 3, 3) = weight_R.block(leg * 3, leg * 3, 3, 3);

                // 计算力误差
                Vector3d f = forceResult.col(leg);  // 使用上一次的计算结果
                Vector3d f_error = f - f_d;         // 力的误差

                // 设置s块（线性项）
                if (step < n_steps - 1)
                {  // 不在最后一步设置s块
                    const int next_force_start = force_start + single_step_dim;
                    mat_H.block(force_start, next_force_start, 3, 3) = -0.5 * weight_R.block(leg * 3, leg * 3, 3, 3);
                    mat_H.block(next_force_start, force_start, 3, 3) = -0.5 * weight_R.block(leg * 3, leg * 3, 3, 3);
                }

                contact_idx++;
            }
        }
    }
}

void GeometricNmpc::set_mat_AB()
{
    // 初始化AB矩阵
    mat_AB.resize(total_dim, total_dim);
    mat_AB.setZero();

    // 对每个预测步设置AB矩阵块
    for (int step = 0; step < n_steps; ++step)
    {
        const int row_start = step * single_step_dim;
        const int col_start = step * single_step_dim;

        // 设置A块
        mat_AB.block(row_start, col_start, state_dim, state_dim) = mat_A;

        // 设置B块
        mat_AB.block(row_start, col_start + state_dim, state_dim, control_dim) = mat_B;

        // 设置单位矩阵I（控制部分）
        mat_AB.block(row_start + state_dim, col_start + state_dim, control_dim, control_dim) =
            Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(control_dim, control_dim);

        // 如果不是最后一步，设置连接下一步的块
        if (step < n_steps - 1)
        {
            // 设置-I块（状态部分）
            mat_AB.block(row_start + single_step_dim, col_start, state_dim, state_dim) =
                -Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(state_dim, state_dim);
        }
    }
}

void GeometricNmpc::run()
{
    // 开始计时
    auto start_time = std::chrono::high_resolution_clock::now();

    // 设置输入和更新维度
    set_input();

    // 设置状态方程矩阵
    set_mat_A();
    set_mat_B();

    // 构建等式约束
    set_mat_AB();
    set_mat_H();
    set_mat_C();
    set_vec_l();
    set_vec_u();
#define USE_SPARSE
#ifndef USE_SPARSE

    // 创建并配置QP求解器
    proxsuite::proxqp::dense::QP<double> qp_dense(total_dim, total_dim, total_constraints);
    qp_dense.settings.eps_abs = 1.0e-6;
    qp_dense.settings.eps_rel = 1.0e-6;
    qp_dense.settings.max_iter = 10;

    // 设置QP问题
    qp_dense.init(mat_H, std::nullopt, mat_AB, std::nullopt, mat_C, vec_l, vec_u);

    // 求解QP问题
    qp_dense.solve();
    const auto& result = qp_dense.results;

#else

    // 创建并配置QP求解器
    proxsuite::proxqp::sparse::QP<double, long long> qp_sparse(total_dim, total_dim, total_constraints);
    qp_sparse.settings.eps_abs = 1.0e-6;
    qp_sparse.settings.eps_rel = 1.0e-6;
    qp_sparse.settings.max_iter = 10;
    qp_sparse.settings.compute_timings = true;
    qp_sparse.settings.verbose = true;
    Eigen::SparseMatrix<double> H_spa(mat_H.rows(), mat_H.cols());
    Eigen::SparseMatrix<double> A_spa(mat_AB.rows(), mat_AB.cols());
    Eigen::SparseMatrix<double> C_spa(mat_C.rows(), mat_C.cols());
    H_spa = mat_H.sparseView();
    A_spa = mat_AB.sparseView();
    C_spa = mat_C.sparseView();

    // 设置QP问题
    qp_sparse.init(H_spa, std::nullopt, A_spa, std::nullopt, C_spa, vec_l, vec_u);

    // 求解QP问题
    qp_sparse.solve();
    const auto& result = qp_sparse.results;

#endif
    // 检查求解状态
    if (result.info.status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED)
    {
        RCLCPP_WARN(rclcpp::get_logger("geometric_nmpc"), "QP solver failed with status: %d", result.info.status);
        return;
    }

    // 提取结果并更新接触力
    const VectorXd& solution = result.x;
    for (int step = 0; step < n_steps; ++step)
    {
        int contact_idx = 0;
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            if (contactLegPhase[leg])
            {
                const int force_start_idx = step * single_step_dim + state_dim + contact_idx * single_leg_dim;
                Vector3d delta_f = solution.segment<3>(force_start_idx);
                forceResult.col(leg) = forceResult.col(leg) + delta_f;
                contact_idx++;
            }
        }
    }

    // 计算总用时
    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time);

    // 打印结果
    if (DEBUG_NMPC)
    {
        // 转换求解器状态为字符串
        std::string status_str;
        switch (result.info.status)
        {
            case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED: status_str = "SOLVED"; break;
            case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED: status_str = "MAX_ITER_REACHED"; break;
            case proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE: status_str = "PRIMAL_INFEASIBLE"; break;
            case proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE: status_str = "DUAL_INFEASIBLE"; break;
            default: status_str = "UNKNOWN";
        }

        // 构建详细的状态信息
        std::stringstream ss;
        ss << "\n==================== NMPC Results ====================\n"
           << "Solver Status    : " << status_str << "\n"
           << "Iterations       : " << result.info.iter << "\n"
           << "Objective Value  : " << result.info.objValue << "\n"
           << "Computation Time : " << duration.count() / 1000.0 << " ms\n"
           << "\nContact Forces:\n";

        // 添加接触力信息
        for (int leg = 0; leg < NUM_LEGS; ++leg)
        {
            if (contactLegPhase[leg])
            {
                ss << "  Leg " << leg << ": [" << std::fixed << std::setprecision(3) << forceResult.col(leg).transpose()
                   << "]\n";
            }
        }
        ss << "====================================================\n";

        DEBUG_PRINT(ss.str());
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("geometric_nmpc"),
                    "QP solved in %ld iter, obj: %.3f, time: %.3f ms",
                    result.info.iter,
                    result.info.objValue,
                    duration.count() / 1000.0);
    }
}

void GeometricNmpc::run_sparse()
{
}

}  // namespace Galileo