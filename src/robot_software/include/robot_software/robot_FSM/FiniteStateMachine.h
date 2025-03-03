#pragma once

#include "Eigen/Dense"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/DataTypes.hpp"
#include "robot_software/robot_utils/GaitSchedule.hpp"
namespace Galileo
{

class FiniteStateMachine
{
public:
    FiniteStateMachine();
    ~FiniteStateMachine();

    void update_input();
    void update_output();
    Eigen::Matrix<int, 4, 1> run();

private:
    double time;
    double timestep = 0.001;
    //    double time_sw[4];
    Eigen::Vector4d time_sw;
    int phase;
    bool isChangePhase = 0;

    Eigen::Matrix<int, 4, 1> lastLegContactState;
    Eigen::Matrix<int, 4, 1> legContactState;
    GaitSchedule currentGait;
    Eigen::Matrix<int, 4, 1> LegPhase;

    void updatePhase();

    bool StepFlag = 0;  // TODO: 需要根据实际情况修改 机器人初始位置记录
    int gaitCmd;

    int lasttag;
    const double touch_rate = 0.9;
    const double left_rate = 1.0;
    DataCenter& dataCenter;
};
}  // namespace Galileo