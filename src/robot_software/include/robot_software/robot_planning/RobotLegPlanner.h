#pragma once
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/GaitSchedule.hpp"
#include "robot_software/robot_utils/QuinticSpline.hpp"
namespace Galileo
{
class RobotLegPlanner
{
public:
    RobotLegPlanner();
    ~RobotLegPlanner();

    // 腿轨迹
    robot_target_trajectory::TargetLegTrajectory legTrajectory;

    // 腿轨迹规划
    void update_leg_trajectory();

private:
    DataCenter &dataCenter;  // 数据中心

    // 计算足端位置
    void raibert_foot_location(const vec3 &v, const vec3 &vd, double T);
    // 计算摆动目标
    void get_swing_target(const mat34 &InitToeLocation,
                          const mat34 &InitToeVelo,
                          const mat34 &TargetToeLocation,
                          const mat34 &TargetToeVelo,
                          double Height,
                          double t,
                          double T,
                          int i);
    // 计算支撑目标
    void get_stance_target(const mat34 &initToeLocation,
                           const mat34 &initToeVelo,
                           const vec3 &targetBaseVelo,
                           double t,
                           double T,
                           int i);

    double kv = 0.06;

    // 新增结构体用于状态管理
    struct LegPlanningState
    {
        Eigen::Matrix<int, 4, 1> phase;
        int gaitCmd;
        int isStep;
        vec3 targetBaseVelo;
        mat34 initToeLocation = mat34::Zero();
        mat34 initToeVelo = mat34::Zero();
        vec3 initCoMPos = vec3::Zero();
        vec3 initCoMVelo = vec3::Zero();
    } state;

    // 新增辅助函数
    LegPlanningState get_current_state();
    // 计算目标速度
    mat34 calculate_target_velocity(const vec3 &targetBaseVelo);
    // 生成腿轨迹
    void generate_leg_trajectories(const LegPlanningState &state, const mat34 &targetToeVelo);
};
}  // namespace Galileo