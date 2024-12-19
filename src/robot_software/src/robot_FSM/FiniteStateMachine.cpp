#include "robot_software/robot_FSM/FiniteStateMachine.h"

#include <rclcpp/rclcpp.hpp>

namespace Galileo
{
Galileo::FiniteStateMachine::FiniteStateMachine()
    : dataCenter(DataCenter::getInstance())
{
    LegPhase.setOnes();
    time_sw.setZero();
    time = 0;
}

Galileo::FiniteStateMachine::~FiniteStateMachine()
{
}

void Galileo::FiniteStateMachine::update_input()
{
    gaitCmd = dataCenter.read<robot_user_cmd::UserCmd>()->gaitCmd;
    // legContactState = dataCenter.read<robot_FSM::Command>()->legPhase;
}
void Galileo::FiniteStateMachine::update_output()
{
    robot_FSM::legState legState;

    legState.legPhase = LegPhase;
    legState.timeSw = time_sw;
    legState.isStep = StepFlag;

    dataCenter.write<robot_FSM::legState>(legState);
}
Eigen::Matrix<int, 4, 1> Galileo::FiniteStateMachine::run()
{
    time += timestep;

    static int tag = 0;
    if (StepFlag == 1) tag = gaitCmd;
    StepFlag = 0;

    if (tag == 0)
        currentGait = GaitSchedules::STAND;
    else if (tag == 1)
        currentGait = GaitSchedules::TROT;
    else if (tag == 2)
        currentGait = GaitSchedules::BOUND;
    else if (tag == 3)
        currentGait = GaitSchedules::FLYTROT;
    else if (tag == 4)
        currentGait = GaitSchedules::PACE;
    else if (tag == 5)
        currentGait = GaitSchedules::STANDTROT;

    if (tag - lasttag != 0)
    {
        time = 0;
        std::cout << "change locomotion = " << tag << std::endl << std::endl;
    }

    if (tag == 0)
    {
        LegPhase.setOnes();
        // time = 0;
        updatePhase();
    }
    else if (tag == 1 || tag == 2 || tag == 4)
    {
        if (time <= touch_rate * currentGait.stand_T)
        {
            LegPhase = currentGait.gaitSequence.row(phase).transpose();
        }
        else if (time > touch_rate * currentGait.stand_T && time <= left_rate * currentGait.stand_T)
        {
            if (legContactState.sum() - lastLegContactState.sum() > 0) updatePhase();

            LegPhase = currentGait.gaitSequence.row(phase).transpose();
        }
        else if (time > left_rate * currentGait.stand_T)
        {
            updatePhase();

            LegPhase = currentGait.gaitSequence.row(phase).transpose();
        }

        time_sw.setConstant(time);
    }
    else if (tag == 3)
    {
        for (int i = 0; i < 4; i++)
        {
            if (currentGait.gaitSequence.row(phase)(i) == 0)
                time_sw(i) += timestep;
            else
                time_sw(i) = 0;
        }

        if (currentGait.gaitSequence.row(phase).sum() == 2)
            if (time > currentGait.stand_T)
            {
                updatePhase();
                LegPhase = currentGait.gaitSequence.row(phase).transpose();
            }
            else
            {
                LegPhase = currentGait.gaitSequence.row(phase).transpose();
            }
        else if (currentGait.gaitSequence.row(phase).sum() == 0)
        {
            LegPhase = currentGait.gaitSequence.row(phase).transpose();
            if (time > ((currentGait.swing_T - currentGait.stand_T)) / 2)
            {
                updatePhase();
                LegPhase = currentGait.gaitSequence.row(phase).transpose();
            }
        }
    }
    else if (tag == 5)
    {
        for (int i = 0; i < 4; i++)
        {
            if (currentGait.gaitSequence.row(phase)(i) == 0)
                time_sw(i) += timestep;
            else
                time_sw(i) = 0;
        }

        if (currentGait.gaitSequence.row(phase).sum() == 2)
            if (time > touch_rate * currentGait.swing_T && time <= left_rate * currentGait.swing_T)
            {
                if (legContactState.sum() - lastLegContactState.sum() > 0) updatePhase();

                LegPhase = currentGait.gaitSequence.row(phase).transpose();
            }
            else if (time > left_rate * currentGait.swing_T)
            {
                updatePhase();

                LegPhase = currentGait.gaitSequence.row(phase).transpose();
            }
            else
            {
                LegPhase = currentGait.gaitSequence.row(phase).transpose();
            }
        else if (currentGait.gaitSequence.row(phase).sum() == 4)
        {
            LegPhase = currentGait.gaitSequence.row(phase).transpose();
            if (time > ((currentGait.stand_T - currentGait.swing_T)) / 2)
            {
                updatePhase();
                LegPhase = currentGait.gaitSequence.row(phase).transpose();
            }
        }
    }
    else if (tag == -1)
    {
        LegPhase.setZero();
    }

    lasttag = tag;
    lastLegContactState = legContactState;

    return LegPhase;
}

void FiniteStateMachine::updatePhase()
{
    phase++;
    time = 0;
    StepFlag = 1;

    if (phase >= currentGait.gaitSequence.rows())
    {
        phase = 0;
    }
}
}  // namespace Galileo
