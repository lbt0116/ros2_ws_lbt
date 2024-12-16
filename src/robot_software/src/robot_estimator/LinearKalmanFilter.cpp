#include "robot_software/robot_estimator/LinearKalmanFilter.h"

#include "robot_software/robot_utils/UtilFunc.h"

namespace Galileo
{

LinearKalmanFilter::LinearKalmanFilter()
{
    // 初始化状态向量
    x_.setZero();

    // 初始化协方差矩阵
    P_.setIdentity() * 1e-3;

    // 初始化测量矩阵
    H_.setZero();
    H_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();  // 角度测量
    H_.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();  // 速度测量

    // 初始化过程噪声
    Q_.setIdentity() * 1e-4;
    Q_.block<3, 3>(0, 0) *= 1e-6;  // 位置过程噪声
    Q_.block<3, 3>(3, 3) *= 1e-4;  // 角度过程噪声
    Q_.block<3, 3>(6, 6) *= 1e-2;  // 速度过程噪声

    // 初始化测量噪声
    R_.setIdentity();
    R_.block<3, 3>(0, 0) *= 1e-3;  // 角度测量噪声
    R_.block<3, 3>(3, 3) *= 1e-2;  // 速度测量噪声

    // 初始化状态转移矩阵
    updateStateTransitionMatrix();
}

void LinearKalmanFilter::updateStateTransitionMatrix()
{
    F_.setIdentity();

    // 位置更新
    F_.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * DT;

    // 速度保���不变
    // 角度保持不变
}

Eigen::Matrix3d LinearKalmanFilter::eulerToRotation(const Eigen::Vector3d& euler)
{
    // 使用ZYX顺序的欧拉角，并调用.matrix()转换为旋转矩阵
    return (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitX())).matrix();
}

void LinearKalmanFilter::predict(const Eigen::Vector3d& acc)
{
    // 获取当前姿态
    Eigen::Vector3d euler = x_.segment<3>(3);
    Eigen::Matrix3d R = eulerToRotation(euler);

    // 更新状态
    x_.segment<3>(0) += x_.segment<3>(6) * DT + 0.5 * (R * acc + gravity_) * DT * DT;  // 位置更新
    x_.segment<3>(6) += (R * acc + gravity_) * DT;                                     // 速度更新

    // 更新协方差
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void LinearKalmanFilter::update(const Eigen::Vector3d& measured_angle,
                                const Eigen::Vector3d& measured_velocity)
{
    // 构造测量向量
    Eigen::Matrix<double, MEAS_DIM, 1> z;
    z << measured_angle, measured_velocity;

    // 计算卡尔曼增益
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K =
        P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();

    // 更新状态
    x_ += K * (z - H_ * x_);

    // ���新协方差
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> I = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    P_ = (I - K * H_) * P_;

    // 标准化角度到[-pi, pi]
    x_.segment<3>(3) = UtilFnc::normalizeAngles(x_.segment<3>(3));
}

void LinearKalmanFilter::run(const Eigen::Vector3d& measured_acc,
                             const Eigen::Vector3d& measured_angle,
                             const Eigen::Vector3d& measured_velocity)
{
    // 预测步骤
    predict(measured_acc);

    // 更新步骤
    update(measured_angle, measured_velocity);
}

}  // namespace Galileo