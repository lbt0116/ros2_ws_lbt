#pragma once

#include "robot_software/robot_utils/DataCenter.hpp"

namespace Galileo
{
class RobotJointPlanner
{
public:
    RobotJointPlanner();
    ~RobotJointPlanner();

    void update_joint_trajectory();
    mat34 IK(const mat34& p);
    robot_target_trajectory::TargetJointTrajectory joint_trajectory;
private:
    DataCenter& dataCenter;

    const double abd_offset = 0.08;  // hip关节偏移
    const double Link1 = 0.2;        // 大腿长度
    const double Link2 = 0.2;        // 小腿长度
};
}  // namespace Galileo
