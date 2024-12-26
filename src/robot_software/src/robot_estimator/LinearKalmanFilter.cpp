#include "robot_software/robot_estimator/LinearKalmanFilter.h"

#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/UtilFunc.h"

namespace Galileo
{

LinearKalmanFilter::LinearKalmanFilter()
    : dataCenter_(DataCenter::getInstance())
{
    // 初始化状态向量
    x_.setZero();

    // 初始化协方差矩阵
    P_.setIdentity();

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

    // 速度保持不变
    // 角度保持不变
}

void LinearKalmanFilter::predict(const Eigen::Vector3d& acc, const Eigen::Matrix3d& R)
{
    // 获取当前姿态
    Eigen::Vector3d euler = x_.segment<3>(3);

    // 更新状态
    Eigen::Vector3d pos_update = x_.segment<3>(6) * DT + 0.5 * (R * acc + gravity_) * DT * DT;
    x_.segment<3>(0) += pos_update;

    Eigen::Vector3d vel_update = (R * acc + gravity_) * DT;
    x_.segment<3>(6) += vel_update;

    // 更新协方差
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void LinearKalmanFilter::update(const Eigen::Vector3d& measured_angle, const Eigen::Vector3d& measured_velocity)
{
    // 构造测量向量
    Eigen::Matrix<double, MEAS_DIM, 1> z;
    z << measured_angle, measured_velocity;

    // 计算卡尔曼增益
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM> K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();

    // 计算创新
    Eigen::Matrix<double, MEAS_DIM, 1> innovation = z - H_ * x_;

    // 更新状态
    x_ += K * innovation;

    // 更新协方差
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> I = Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity();
    P_ = (I - K * H_) * P_;
}

void LinearKalmanFilter::run()
{
    // 从DataCenter读取数据
    auto baseState = dataCenter_.read<robot_state::BaseState>();
    auto phase = dataCenter_.read<robot_FSM::legState>()->legPhase;
    auto legVeloInWorld = dataCenter_.read<robot_state::LegState>()->legVeloInWorld;
    auto legPosInWorld = dataCenter_.read<robot_state::LegState>()->legPosHipInWorld;

    // 使用DataCenter中的数据
    Eigen::Vector3d acc = baseState->acceleration;
    Eigen::Vector3d angle = baseState->eulerAngles;
    Eigen::Matrix3d Rot = baseState->rotationMatrix;
    Eigen::Vector3d velocity = -legVeloInWorld * phase.cast<double>() / phase.sum();
    
    if (phase.sum() == 0) velocity = Eigen::Vector3d::Zero();

    predict(acc, Rot);
    update(angle, velocity);
    Eigen::Vector3d x_j = -legPosInWorld * phase.cast<double>() / phase.sum();
    x_(2) = x_j(2);
    if (phase.sum() == 0) x_(2) = 0;

    baseState->positionRelative = x_j;

    baseState->position = x_.segment<3>(0);
    baseState->eulerAngles = x_.segment<3>(3);
    baseState->linearVelocity = x_.segment<3>(6);

    dataCenter_.write(baseState);
}
}  // namespace Galileo
