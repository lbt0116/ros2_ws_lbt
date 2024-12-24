#pragma once

#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/DataTypes.hpp"
#include "robot_software/third_party/quadprog/QuadProg.h"

namespace Galileo
{

class BallanceController
{
public:
    BallanceController();
    ~BallanceController();

    void run();
    Eigen::VectorXd F_ext;
    // 声明参数
    void declare_and_get_parameters(rclcpp::Node* node);

    void set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    double w = 1e-3;

private:
    struct QPMatrices
    {
        matxd A;
        matxd B;
        matxd C;
        vecxd d;
    };

    double FORCE_MAX = 1000.0;
    double FORCE_MIN = -25.0;

    DataCenter& dataCenter_;
    vec6 kp{1, 1, 1, 1, 1, 1};
    vec6 kd{1, 1, 1, 1, 1, 1};
    vec6 s_vec{1, 1, 1, 1, 1, 1};

    // 辅助函数
    std::tuple<vec12, vec12> get_current_and_target_state() const;
    vec6 compute_balance_force(const vec12& state, const vec12& state_des) const;
    Eigen::Matrix<double, 6, 3> build_friction_cone_constraints() const;
};

}  // namespace Galileo
