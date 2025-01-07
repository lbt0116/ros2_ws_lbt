#pragma once
#include "robot_software/robot_utils/DataCenter.hpp"
namespace Galileo
{
class RobotBasePlanner
{
public:
    RobotBasePlanner();
    ~RobotBasePlanner();
    void update_base_trajectory();

private:
    /**
     * @brief 根据当前四元数和角速度指令更新目标四元数
     * @param currentQuat 当前四元数
     * @param angularVel 角速度指令
     * @param dt 时间步长
     * @return 目标四元数
     */
    Eigen::Quaterniond updateTargetQuaternion(const Eigen::Quaterniond& currentQuat,
                                              const Eigen::Vector3d& angularVel,
                                              double dt);

    DataCenter& dataCenter;  // 数据中心

    // 基座轨迹
    robot_target_trajectory::TargetBaseTrajectory baseTrajectory;
    long double time;
};
}  // namespace Galileo
