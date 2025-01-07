#include "robot_software/robot_planning/RobotLegPlanner.h"

namespace Galileo
{

RobotLegPlanner::RobotLegPlanner()
    : dataCenter(DataCenter::getInstance())
{
    state.initToeLocation << 0, 0, 0, 0, 0, 0, 0, 0, -0.5, -0.5, -0.5, -0.5;
    // legTrajectory.targetLegPosition << 0, 0, -0.5, 0, 0, -0.5, 0, 0, -0.5, 0, 0, -0.5;
}

RobotLegPlanner::~RobotLegPlanner()
{
}

void RobotLegPlanner::update_leg_trajectory()
{
    // 1. 获取当前状态
    LegPlanningState state = get_current_state();

    // 2. 计算目标落足点
    raibert_foot_location(state.initCoMVelo, state.targetBaseVelo, GaitSchedules::getGaitByTag(state.gaitCmd).swing_T);

    // 3. 计算目标速度
    mat34 targetToeVelo = calculate_target_velocity(state.targetBaseVelo);

    // 4. 生成各条腿的轨迹
    generate_leg_trajectories(state, targetToeVelo);

    // 5. 写入数据中心
    dataCenter.write(legTrajectory);
}

RobotLegPlanner::LegPlanningState RobotLegPlanner::get_current_state()
{
    // 1. 获取当前状态
    // LegPlanningState state;

    // 2. 获取当前步态
    state.phase = dataCenter.read<robot_FSM::legState>()->legPhase;
    state.isStep = dataCenter.read<robot_FSM::legState>()->isStep;

    // 3. 获取当前用户指令
    state.gaitCmd = dataCenter.read<robot_user_cmd::UserCmd>()->gaitCmd;
    state.targetBaseVelo = dataCenter.read<robot_user_cmd::UserCmd>()->veloCmd;

    // 4. 获取当前步态状态
    if (state.isStep == 1)
    {
        state.initToeLocation = dataCenter.read<robot_state::LegState>()->legPosHipInWorld;
        state.initToeVelo = dataCenter.read<robot_state::LegState>()->legVeloInWorld;
        state.initCoMPos = dataCenter.read<robot_state::BaseState>()->position;
        state.initCoMVelo = dataCenter.read<robot_state::BaseState>()->linearVelocity;
    }

    return state;
}

mat34 RobotLegPlanner::calculate_target_velocity(const vec3 &targetBaseVelo)
{
    mat34 targetToeVelo;
    targetToeVelo.setZero();
    targetToeVelo.row(0).setConstant(-targetBaseVelo(0));
    targetToeVelo.row(1).setConstant(-targetBaseVelo(1));
    return targetToeVelo;
}

void RobotLegPlanner::generate_leg_trajectories(const LegPlanningState &state, const mat34 &targetToeVelo)
{
    const auto &gaitSchedule = GaitSchedules::getGaitByTag(state.gaitCmd);

    for (int i = 0; i < 4; i++)
    {
        if (state.phase(i) == 0)  // 摆动相
        {
            get_swing_target(state.initToeLocation,
                             state.initToeVelo,
                             legTrajectory.targetToeLocation,
                             targetToeVelo,
                             gaitSchedule.swingHeight,
                             dataCenter.read<robot_FSM::legState>()->timeSw(i),
                             gaitSchedule.swing_T,
                             i);
        }
        else  // 支撑相
        {
            get_stance_target(state.initToeLocation,
                              state.initToeVelo,
                              state.targetBaseVelo,
                              dataCenter.read<robot_FSM::legState>()->timeSw(i),
                              gaitSchedule.stand_T,
                              i);
        }
    }
}

void RobotLegPlanner::raibert_foot_location(const vec3 &v, const vec3 &vd, double T)
{
    // 获取机器人指令
    auto tarW = dataCenter.read<robot_user_cmd::UserCmd>()->angveloCmd;
    // 获取机器人参数
    auto baseLength = dataCenter.read<robot_constants>()->length;
    auto baseWidth = dataCenter.read<robot_constants>()->width;
    auto xOffset = dataCenter.read<robot_constants>()->xOffset;
    auto yOffset = dataCenter.read<robot_constants>()->yOffset;

    // 计算偏移量
    if (abs(vd(0)) < 0.01)
    {
        xOffset = 0;
    }

    // 计算yaw旋转
    const Eigen::AngleAxisd yaw(tarW(2) * T, Eigen::Vector3d::UnitZ());

    // 定义基础足端位置矩阵 (3×4)
    Eigen::Matrix<double, 3, 4> Base;
    // 每列代表一条腿的[x, y, z]坐标
    Base << baseLength / 2, -baseLength / 2, baseLength / 2, -baseLength / 2,  // X坐标
        -baseWidth / 2, -baseWidth / 2, baseWidth / 2, baseWidth / 2,          // Y坐标
        0, 0, 0, 0;                                                            // Z坐标

    // 计算转向补偿 (3×4)
    Eigen::Matrix<double, 3, 4> turn = yaw.toRotationMatrix() * Base - Base;

    // 计算目标位置
    for (int i = 0; i < 4; i++)
    {
        // X方向
        double xOff = (i == 0 || i == 2) ? xOffset : -xOffset;
        legTrajectory.targetToeLocation(0, i) = 0.5 * v(0) * T + kv * (v(0) - vd(0)) + xOff;

        // Y方向
        double yOff = (i < 2) ? -yOffset : yOffset;
        legTrajectory.targetToeLocation(1, i) = 0.5 * v(1) * T + kv * (v(1) - vd(1)) + yOff;

        // Z方向
        legTrajectory.targetToeLocation(2, i) = 0;
    }

    legTrajectory.targetToeLocation += turn;

    // 设置目标速度
    // legTrajectory.v.setZero();
    // legTrajectory.v.row(0).setConstant(
    //     -dataCenter.read<robot_user_cmd::UserCmd>()->veloCmd(0));
    // TODO: 维度检查
}

void RobotLegPlanner::get_swing_target(const mat34 &initToeLocation,
                                       const mat34 &initToeVelo,
                                       const mat34 &targetToeLocation,
                                       const mat34 &targetToeVelo,
                                       double height,
                                       double t,
                                       double T,
                                       int i)
{
    // X方向轨迹
    QuinticTrajectory trajX(initToeLocation(0, i),                                  // p0
                            initToeVelo(0, i),                                      // v0
                            0,                                                      // a0
                            0.5 * T,                                                // tm
                            (initToeLocation(0, i) + targetToeLocation(0, i)) / 2,  // pm
                            targetToeLocation(0, i),                                // pf
                            targetToeVelo(0, i),                                    // vf
                            0,                                                      // af
                            T);                                                     // T

    // Y方向轨迹
    QuinticTrajectory trajY(initToeLocation(1, i),
                            initToeVelo(1, i),
                            0,
                            0.5 * T,
                            (initToeLocation(1, i) + targetToeLocation(1, i)) / 2,
                            targetToeLocation(1, i),
                            targetToeVelo(1, i),
                            0,
                            T);

    // Z方向轨迹
    double finalZ = initToeLocation(2, i);
    double midZ = initToeLocation(2, i) + height;

    // 考虑倾斜角度的情况
    // if (tarQ(1) != 0)
    // {
    //     double sw_H = (TargetToeLocation(0, i) - InitToeLocation(0, i)) * sin(tarQ(1));
    //     finalZ = InitToeLocation(2, i) - sw_H;
    // }

    QuinticTrajectory trajZ(initToeLocation(2, i),  // 起始高度
                            0,                      // 初始速度
                            0,                      // 初始加速度
                            0.5 * T,                // 中间时刻
                            midZ,                   // 最高点
                            finalZ,                 // 终止高度
                            targetToeVelo(2, i),    // 终止速度
                            0,                      // 终止加速度
                            T);

    // 获取当前时刻的位置、速度和加速度
    trajX.evaluate(t,
                   legTrajectory.targetLegPosition(0, i),
                   legTrajectory.targetLegVelocity(0, i),
                   legTrajectory.targetLegAcceleration(0, i));
    trajY.evaluate(t,
                   legTrajectory.targetLegPosition(1, i),
                   legTrajectory.targetLegVelocity(1, i),
                   legTrajectory.targetLegAcceleration(1, i));
    trajZ.evaluate(t,
                   legTrajectory.targetLegPosition(2, i),
                   legTrajectory.targetLegVelocity(2, i),
                   legTrajectory.targetLegAcceleration(2, i));
}

void RobotLegPlanner::get_stance_target(const mat34 &initToeLocation,
                                        const mat34 &initToeVelo,
                                        const vec3 &targetBaseVelo,
                                        double t,
                                        double T,
                                        int i)
{
    // 支撑腿的轨迹为base期望速度的取反的线性积分
    // p = p0 + (-v_base) * t
    // v = -v_base
    // a = 0

    // 位置：线性积分
    legTrajectory.targetLegPosition(0, i) = initToeLocation(0, i) - targetBaseVelo(0) * t;
    legTrajectory.targetLegPosition(1, i) = initToeLocation(1, i) - targetBaseVelo(1) * t;
    legTrajectory.targetLegPosition(2, i) = initToeLocation(2, i);  // Z方向保持不变

    // 速度：与base期望速度相反
    legTrajectory.targetLegVelocity(0, i) = -targetBaseVelo(0);
    legTrajectory.targetLegVelocity(1, i) = -targetBaseVelo(1);
    legTrajectory.targetLegVelocity(2, i) = 0.0;  // Z方向速度为0

    // 加速度：全部为0
    legTrajectory.targetLegAcceleration(0, i) = 0.0;
    legTrajectory.targetLegAcceleration(1, i) = 0.0;
    legTrajectory.targetLegAcceleration(2, i) = 0.0;
}

}  // namespace Galileo
