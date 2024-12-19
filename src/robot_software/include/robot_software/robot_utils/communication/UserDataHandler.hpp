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

        msg.leg_pose = eigenToStdArray(dataCenter_.read<robot_state::LegState>()->legPosHipInWorld);
        msg.leg_pose_des =
            eigenToStdArray(dataCenter_.read<robot_target_trajectory::TargetLegTrajectory>()->p);
        msg.gait_cmd = dataCenter_.read<robot_user_cmd::UserCmd>()->gaitCmd;
        msg.base_vel_des = eigenToStdArray(dataCenter_.read<robot_user_cmd::UserCmd>()->veloCmd);
        msg.leg_phase = eigenToStdArray(dataCenter_.read<robot_FSM::legState>()->legPhase);
        return msg;
    }

private:
    DataCenter& dataCenter_;
};
}  // namespace Galileo
