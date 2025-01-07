#pragma once
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_utils/DataCenter.hpp"
namespace Galileo
{
class GeometricNmpc
{
public:
    GeometricNmpc();
    ~GeometricNmpc() = default;

    void run();
    void run_sparse();

    void declare_and_get_parameters(rclcpp::Node *node);

    void set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

private:
    // MPC parameters
    double dt_MPC;
    int n_steps;
    double u;
    double fmax;
    double fmin;

    // physical parameter
    double mass;
    vec3 gravity;
    mat33 Inertia;
    mat66 Jb;
    mat66 Jb_inv;

    // Weight matrices (动态大小)
    Eigen::MatrixXd weight_Q;
    Eigen::MatrixXd weight_R;
    Eigen::MatrixXd weight_P;

    // 状态向量
    vec3 p;
    mat33 R;
    vec3 v;
    vec3 w;

    vec3 p_d;
    mat33 R_d;
    vec3 v_d;
    vec3 w_d;

    mat44 SE3;
    mat44 SE3_d;

    vec6 se3;
    vec6 se3_d;

    vec4i contactLegPhase;
    mat34 contactLegPos;
    mat34 forceResult;
    // QP solver
    matxd mat_H;   // Hessian 矩阵
    matxd mat_A;   // 状态方程A矩阵
    matxd mat_B;   // 状态方程B矩阵
    matxd mat_AB;  // 等式约束矩阵
    matxd mat_C;   // 不等式约束矩阵
    vecxd vec_l;   // 不等式约束下界
    vecxd vec_u;   // 不等式约束上界

    //
    // function
    void set_input();
    void set_mat_A();
    void set_mat_B();
    void set_mat_C();
    void set_vec_l();
    void set_vec_u();
    void set_mat_H();
    void set_mat_AB();
    void update_dimensions();
    DataCenter &dataCenter_;

    // 维度相关变量
    static constexpr int state_dim = 12;      // 状态维度（固定为12）
    static constexpr int single_leg_dim = 3;  // 单腿力的维度（固定为3）
    int num_contact;                          // 接触腿的数量
    int control_dim;                          // 控制维度（num_contact * single_leg_dim）
    int single_step_dim;                      // 单步总维度（state_dim + control_dim）
    int total_dim;                            // 总维度（single_step_dim * n_steps）
    int constraints_per_step;                 // 每步的约束数量（num_contact * single_leg_dim）
    int total_constraints;                    // 总约束数量（constraints_per_step * n_steps）
};
}  // namespace Galileo
