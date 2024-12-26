#include "robot_software/robot_planning/RobotBasePlanner.h"

namespace Galileo
{
RobotBasePlanner::RobotBasePlanner()
    : dataCenter(DataCenter::getInstance())
{
    
}

RobotBasePlanner::~RobotBasePlanner()
{
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

    baseTrajectory.targetPosition += v_d * 0.001;

    baseTrajectory.targetPosition(2) = std::clamp(baseTrajectory.targetPosition(2), 0.1, 0.6);

    /*angular update*/
    auto w_d = dataCenter.read<robot_user_cmd::UserCmd>()->angveloCmd;

    w_d(0) = std::clamp(w_d(0), -1.0, 1.0);
    w_d(1) = std::clamp(w_d(1), -1.0, 1.0);
    w_d(2) = std::clamp(w_d(2), -2.0, 2.0);

    baseTrajectory.targetAngularVelocity = updateVelocity(baseTrajectory.targetAngularVelocity, w_d, 2);  // 5 rad/s^2

    baseTrajectory.targetEulerAngles += w_d * 0.001;

    baseTrajectory.targetEulerAngles(0) = std::clamp(baseTrajectory.targetEulerAngles(0), -2 * M_PI / 3, 2 * M_PI / 3);
    baseTrajectory.targetEulerAngles(1) = std::clamp(baseTrajectory.targetEulerAngles(1), -2 * M_PI / 3, 2 * M_PI / 3);

    dataCenter.write(baseTrajectory);
}

}  // namespace Galileo
