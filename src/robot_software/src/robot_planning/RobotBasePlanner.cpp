#include "robot_software/robot_planning/RobotBasePlanner.h"

#include "robot_software/robot_utils/DataTypes.hpp"

namespace Galileo
{
RobotBasePlanner::RobotBasePlanner()
    : dataCenter(DataCenter::getInstance())
{
}

RobotBasePlanner::~RobotBasePlanner()
{
}

Eigen::Quaterniond RobotBasePlanner::updateTargetQuaternion(const Eigen::Quaterniond& currentQuat,
                                                            const Eigen::Vector3d& angularVel,
                                                            double dt)
{
    // 将角速度转换为四元数变化率
    Eigen::Quaterniond omega;
    omega.w() = 0;
    omega.vec() = angularVel;

    // 计算四元数的导数: dq/dt = 1/2 * q * omega
    Eigen::Quaterniond qDot;
    qDot.w() = -0.5 * (omega.x() * currentQuat.x() + omega.y() * currentQuat.y() + omega.z() * currentQuat.z());
    qDot.x() = 0.5
               * (omega.w() * currentQuat.x() + omega.z() * currentQuat.y() - omega.y() * currentQuat.z()
                  + omega.x() * currentQuat.w());
    qDot.y() = 0.5
               * (-omega.z() * currentQuat.x() + omega.w() * currentQuat.y() + omega.x() * currentQuat.z()
                  + omega.y() * currentQuat.w());
    qDot.z() = 0.5
               * (omega.y() * currentQuat.x() - omega.x() * currentQuat.y() + omega.w() * currentQuat.z()
                  + omega.z() * currentQuat.w());

    // 使用欧拉积分计算下一时刻的四元数
    Eigen::Quaterniond targetQuat;
    targetQuat.w() = currentQuat.w() + qDot.w() * dt;
    targetQuat.x() = currentQuat.x() + qDot.x() * dt;
    targetQuat.y() = currentQuat.y() + qDot.y() * dt;
    targetQuat.z() = currentQuat.z() + qDot.z() * dt;

    // 归一化四元数
    targetQuat.normalize();

    return targetQuat;
}

void RobotBasePlanner::update_base_trajectory()
{
    /*linear update*/
    auto v_d = dataCenter.read<robot_user_cmd::UserCmd>()->veloCmd;

    v_d(0) = std::clamp(v_d(0), -3.0, 3.0);
    v_d(1) = std::clamp(v_d(1), -2.0, 2.0);
    v_d(2) = std::clamp(v_d(2), -1.0, 1.0);

    // 定义一个 lambda 函数，计算下一步的速度
    auto updateVelocity = [&](const Eigen::Vector3d& current,
                              const Eigen::Vector3d& target,
                              const double& acceleration) -> Eigen::Vector3d
    {
        Eigen::Vector3d diff = target - current;  // 速度差
        double max_delta = acceleration * 0.001;  // 本时间步内的变化限幅

        // 如果速度差的模小于本步最大变化量，直接返回目标速度
        if (diff.norm() <= max_delta)
        {
            return target;
        }

        // 否则，按速度差的方向加速，限幅到 max_delta
        return current + diff.normalized() * max_delta;
    };

    baseTrajectory.targetLinearVelocity = updateVelocity(baseTrajectory.targetLinearVelocity, v_d, 10);

    baseTrajectory.targetPosition += baseTrajectory.targetLinearVelocity * 0.001;

    baseTrajectory.targetPosition(2) = std::clamp(baseTrajectory.targetPosition(2), 0.1, 0.6);

    /*angular update*/
    auto w_d = dataCenter.read<robot_user_cmd::UserCmd>()->angveloCmd;

    w_d(0) = std::clamp(w_d(0), -1.0, 1.0);
    w_d(1) = std::clamp(w_d(1), -1.0, 1.0);
    w_d(2) = std::clamp(w_d(2), -2.0, 2.0);

    baseTrajectory.targetAngularVelocity = updateVelocity(baseTrajectory.targetAngularVelocity, w_d, 2);  // 5 rad/s^2

    // 更新目标四元数

    baseTrajectory.targetQuaternion =
        updateTargetQuaternion(baseTrajectory.targetQuaternion, baseTrajectory.targetAngularVelocity, 0.001);

    // baseTrajectory.targetQuaternion = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());

    dataCenter.write(baseTrajectory);
}

}  // namespace Galileo
