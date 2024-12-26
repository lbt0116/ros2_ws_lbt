#include "robot_software/robot_planning/RobotJointPlanner.h"

#include <cmath>

namespace Galileo
{
RobotJointPlanner::RobotJointPlanner()
    : dataCenter(DataCenter::getInstance())
{
    joint_trajectory.targetJointPosition.col(0) << 0, -M_PI / 4, M_PI / 2;
    joint_trajectory.targetJointPosition.col(1) << 0, -M_PI / 4, M_PI / 2;
    joint_trajectory.targetJointPosition.col(2) << 0, -M_PI / 4, M_PI / 2;
    joint_trajectory.targetJointPosition.col(3) << 0, -M_PI / 4, M_PI / 2;
}

RobotJointPlanner::~RobotJointPlanner()
{
}

void RobotJointPlanner::update_joint_trajectory()
{
    joint_trajectory.targetJointVelocity << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

    dataCenter.write<robot_target_trajectory::TargetJointTrajectory>(joint_trajectory);
}

mat34 RobotJointPlanner::IK(const mat34& p)
{
    mat34 q;

    // 优化：提前计算常用值
    auto compute_leg_ik = [this](const vec3& pos, bool is_left_leg)
    {
        vec3 angles;
        const double L = std::hypot(pos(1), pos(2));
        const double L1 = std::sqrt(L * L - abd_offset * abd_offset);
        const double L2 = std::hypot(L1, pos(0));

        // 计算hip angle (abduction/adduction)
        const double hip_angle_base = std::acos(abd_offset / L);
        const double hip_angle_offset = std::asin(pos(2) / L);

        if (is_left_leg)
        {
            angles(0) =
                (pos(1) < 0) ? -(hip_angle_base + hip_angle_offset) : M_PI - (hip_angle_base - hip_angle_offset);
        }
        else
        {
            angles(0) =
                (pos(1) > 0) ? (hip_angle_base + hip_angle_offset) : -(M_PI - (hip_angle_base - hip_angle_offset));
        }

        // 计算knee和ankle angles
        const double leg_angle = std::acos(L2 / (2 * Link1));
        angles(1) = std::atan2(pos(0), L1) - leg_angle;
        angles(2) = 2 * leg_angle;

        return angles;
    };

    // 计算左前腿和左后腿
    for (int i = 0; i < 2; i++)
    {
        vec3 pos = p.col(i);
        q.col(i) = compute_leg_ik(pos, true);
    }

    // 计算右前腿和右后腿
    for (int i = 2; i < 4; i++)
    {
        vec3 pos = p.col(i);
        q.col(i) = compute_leg_ik(pos, false);
    }

    return q;
}

}  // namespace Galileo