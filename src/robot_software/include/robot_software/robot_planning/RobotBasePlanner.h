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
    DataCenter &dataCenter;  // 数据中心

    // 基座轨迹
    robot_target_trajectory::TargetBaseTrajectory baseTrajectory;
    long double time;
};
}  // namespace Galileo
