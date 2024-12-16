#ifndef LINEAR_KALMAN_FILTER_H
#define LINEAR_KALMAN_FILTER_H

#include <Eigen/Dense>

#include "robot_software/robot_interface/PinocchioInterface.h"

namespace Galileo
{

class LinearKalmanFilter
{
private:
    static constexpr int STATE_DIM = 9;  // [position(3), angle(3), velocity(3)]
    static constexpr int MEAS_DIM = 6;   // [angle(3), velocity(3)]
    static constexpr double DT = 0.001;  // 时间步长

    // 状态向量和矩阵
    Eigen::Matrix<double, STATE_DIM, 1> x_;          // 状态向量 [p, theta, v]
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_;  // 状态协方差
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F_;  // 状态转移矩阵
    Eigen::Matrix<double, MEAS_DIM, STATE_DIM> H_;   // 观测矩阵

    // 噪声协方差
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q_;  // 过程噪声
    Eigen::Matrix<double, MEAS_DIM, MEAS_DIM> R_;    // 测量噪声

    // 重力向量
    const Eigen::Vector3d gravity_{0, 0, -9.81};

    // 更新状态转移矩阵
    void updateStateTransitionMatrix();

    // 从欧拉角计算旋转矩阵
    Eigen::Matrix3d eulerToRotation(const Eigen::Vector3d& euler);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinearKalmanFilter();

    // 预测步骤
    void predict(const Eigen::Vector3d& acc);

    // 更新步骤
    void update(const Eigen::Vector3d& measured_angle,
                const Eigen::Vector3d& measured_velocity);

    // 获取估计状态
    Eigen::Vector3d getPosition() const { return x_.segment<3>(0); }
    Eigen::Vector3d getAngle() const { return x_.segment<3>(3); }
    Eigen::Vector3d getVelocity() const { return x_.segment<3>(6); }

    // 统一的入口函数
    void run(const Eigen::Vector3d& measured_acc,
             const Eigen::Vector3d& measured_angle,
             const Eigen::Vector3d& measured_velocity);
};

}  // namespace Galileo

#endif  // LINEAR_KALMAN_FILTER_H