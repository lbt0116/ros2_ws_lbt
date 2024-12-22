#pragma once

#include "custom_msgs/msg/to_ui_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/UtilFunc.h"
namespace Galileo
{
class UserDataHandler
{
public:
    UserDataHandler()
        : dataCenter_(DataCenter::getInstance()){};
    ~UserDataHandler(){};

    custom_msgs::msg::ToUiMsg updateUserData()
    {
        custom_msgs::msg::ToUiMsg msg;
        // leg
        msg.leg_pos = eigenToStdArray(dataCenter_.read<robot_state::LegState>()->legPosHipInWorld);

        msg.leg_pos_des = eigenToStdArray(dataCenter_.read<robot_target_trajectory::TargetLegTrajectory>()->p);

        msg.gait_cmd = dataCenter_.read<robot_user_cmd::UserCmd>()->gaitCmd;

        msg.leg_phase = eigenToStdArray(dataCenter_.read<robot_FSM::legState>()->legPhase);

        // base
        msg.base_pos_des =
            eigenToStdArray(dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetPosition);

        msg.base_ang_des =
            eigenToStdArray(dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetEulerAngles);

        msg.base_vel_des =
            eigenToStdArray(dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetLinearVelocity);

        msg.base_ang_vel_des =
            eigenToStdArray(dataCenter_.read<robot_target_trajectory::TargetBaseTrajectory>()->targetAngularVelocity);

        return msg;
    }

private:
    DataCenter& dataCenter_;
};
}  // namespace Galileo
