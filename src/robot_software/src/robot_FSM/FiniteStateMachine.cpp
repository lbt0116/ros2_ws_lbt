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

    stand.gaitSequence.resize(1, 4);
    stand.gaitSequence << 1, 1, 1, 1;
    stand.gaitTag = 0;
    stand.stand_T = 1;
    stand.swing_T = 0;
    stand.standHeight = 0.55;

    trot.gaitSequence.resize(2, 4);
    trot.gaitSequence << 1, 0, 0, 1, 0, 1, 1, 0;
    trot.gaitTag = 1;
    trot.stand_T = 0.2;
    trot.swing_T = 0.2;
    trot.swingHeight = 0.1;
    trot.standHeight = 0.55;

    bound.gaitSequence.resize(2, 4);
    bound.gaitSequence << 0, 1, 0, 1, 1, 0, 1, 0;
    bound.gaitTag = 2;
    bound.stand_T = 0.25;
    bound.swing_T = 0.25;
    bound.swingHeight = 0.08;

    pace.gaitSequence.resize(2, 4);
    pace.gaitSequence << 1, 1, 0, 0, 0, 0, 1, 1;
    pace.gaitTag = 3;
    pace.stand_T = 0.25;
    pace.swing_T = 0.25;
    pace.swingHeight = 0.08;

    flytrot.gaitSequence.resize(4, 4);
    flytrot.gaitSequence << 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0;
    flytrot.gaitTag = 4;
    flytrot.stand_T = 0.2;
    flytrot.swing_T = 0.3;
    flytrot.swingHeight = 0.08;
    flytrot.standHeight = 0.65;

    standtrot.gaitSequence.resize(4, 4);
    standtrot.gaitSequence << 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1;
    standtrot.gaitTag = 5;
    standtrot.stand_T = 0.3;
    standtrot.swing_T = 0.25;
    standtrot.swingHeight = 0.15;  // taijiaogaodu
    standtrot.standHeight = 0.55;
}

Galileo::FiniteStateMachine::~FiniteStateMachine()
{
}

void Galileo::FiniteStateMachine::update_input()
{
    gaitCmd = dataCenter.read<robot_FSM::Command>()->gaitCmd;
    // legContactState = dataCenter.read<robot_FSM::Command>()->legPhase;
}
void Galileo::FiniteStateMachine::update_output()
{
    robot_FSM::Command Command;
    Command.gaitCmd = gaitCmd;
    Command.legPhase = LegPhase;
    dataCenter.write<robot_FSM::Command>(Command);
}
Eigen::Matrix<int, 4, 1> Galileo::FiniteStateMachine::run()
{
    time += timestep;

    static int tag = 0;
    if (StepFlag == 1) tag = gaitCmd;
    StepFlag = 0;

    if (tag == 0)
        currentGait = &stand;
    else if (tag == 1)
        currentGait = &trot;
    else if (tag == 2)
        currentGait = &bound;
    else if (tag == 3)
        currentGait = &flytrot;
    else if (tag == 4)
        currentGait = &pace;
    else if (tag == 5)
        currentGait = &standtrot;

    if (tag - lasttag != 0)
    {
        time = 0;
        std::cout << "change locomotion = " << tag << std::endl << std::endl;
    }

    if (tag == 0)
    {
        LegPhase.setOnes();
        updatePhase();
    }
    else if (tag == 1 || tag == 2 || tag == 4)
    {
        if (time <= touch_rate * currentGait->stand_T)
        {
            LegPhase = currentGait->gaitSequence.row(phase).transpose();
        }
        else if (time > touch_rate * currentGait->stand_T
                 && time <= left_rate * currentGait->stand_T)
        {
            if (legContactState.sum() - lastLegContactState.sum() > 0) updatePhase();

            LegPhase = currentGait->gaitSequence.row(phase).transpose();
        }
        else if (time > left_rate * currentGait->stand_T)
        {
            updatePhase();

            LegPhase = currentGait->gaitSequence.row(phase).transpose();
        }

        time_sw.setConstant(time);
    }
    else if (tag == 3)
    {
        for (int i = 0; i < 4; i++)
        {
            if (currentGait->gaitSequence.row(phase)(i) == 0)
                time_sw(i) += timestep;
            else
                time_sw(i) = 0;
        }

        if (currentGait->gaitSequence.row(phase).sum() == 2)
            if (time > currentGait->stand_T)
            {
                updatePhase();
                LegPhase = currentGait->gaitSequence.row(phase).transpose();
            }
            else
            {
                LegPhase = currentGait->gaitSequence.row(phase).transpose();
            }
        else if (currentGait->gaitSequence.row(phase).sum() == 0)
        {
            LegPhase = currentGait->gaitSequence.row(phase).transpose();
            if (time > ((currentGait->swing_T - currentGait->stand_T)) / 2)
            {
                updatePhase();
                LegPhase = currentGait->gaitSequence.row(phase).transpose();
            }
        }
    }
    else if (tag == 5)
    {
        for (int i = 0; i < 4; i++)
        {
            if (currentGait->gaitSequence.row(phase)(i) == 0)
                time_sw(i) += timestep;
            else
                time_sw(i) = 0;
        }

        if (currentGait->gaitSequence.row(phase).sum() == 2)
            if (time > touch_rate * currentGait->swing_T
                && time <= left_rate * currentGait->swing_T)
            {
                if (legContactState.sum() - lastLegContactState.sum() > 0) updatePhase();

                LegPhase = currentGait->gaitSequence.row(phase).transpose();
            }
            else if (time > left_rate * currentGait->swing_T)
            {
                updatePhase();

                LegPhase = currentGait->gaitSequence.row(phase).transpose();
            }
            else
            {
                LegPhase = currentGait->gaitSequence.row(phase).transpose();
            }
        else if (currentGait->gaitSequence.row(phase).sum() == 4)
        {
            LegPhase = currentGait->gaitSequence.row(phase).transpose();
            if (time > ((currentGait->stand_T - currentGait->swing_T)) / 2)
            {
                updatePhase();
                LegPhase = currentGait->gaitSequence.row(phase).transpose();
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

    if (phase >= currentGait->gaitSequence.rows())
    {
        phase = 0;
    }
}
}  // namespace Galileo
