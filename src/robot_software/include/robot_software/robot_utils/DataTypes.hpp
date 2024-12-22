#pragma once
#include <Eigen/Dense>
#include <array>

#include "robot_software/robot_utils/MatrixTypes.h"

namespace Galileo
{

struct robot_constants
{
    double mass = 50;
    double length = 0.6;
    double width = 0.4;
    double legLength = 0.36;
    double xOffset = 0.06;
    double yOffset = 0.08;
    mat33 inertiaMatrix;
    mat66 spatialInertiaMatrix;
};

// 机器人状态相关数据结构
namespace robot_state
{
struct SensorData
{
    mat34 jointPosition;
    mat34 jointVelocity;
    mat34 jointTorque;
    vec4 imuQuant;
    vec3 imuAngvelo;
    vec3 imuAcc;
};

struct JointState
{
    mat34 jointPosition;
    mat34 jointVelocity;
    mat34 jointTorque;
};

struct LegState
{
    mat34 legPosHipInBody;
    mat34 legPosHipInWorld;
    mat34 legPosBaseInBody;
    mat34 legPosBaseInWorld;
    mat34 legVeloInBody;
    mat34 legVeloInWorld;
    mat34 legForceInBody;
    mat34 legForceInWorld;
    Eigen::Matrix<double, 6, 18> legJacobian[4];
};

struct BaseState
{
    vec3 position;
    vec3 eulerAngles;      // 欧拉角
    mat33 rotationMatrix;  // 旋转矩阵
    vec4 quaternion;       // 四元数
    vec3 linearVelocity;   // 线速度
    vec3 angularVelocity;  // 角速度

    vec3 acceleration;
};

struct ContactState
{
    vec4i isContact;
    mat34 contactForce;
};
}  // namespace robot_state

namespace robot_FSM
{

struct legState
{
    Eigen::Matrix<int, 4, 1> legPhase;
    int isStep;
    Eigen::Matrix<double, 4, 1> timeSw;
};

}  // namespace robot_FSM

namespace robot_target_trajectory
{

struct TargetBaseTrajectory
{
    vec3 targetPosition = {0, 0, 0.5};
    vec3 targetEulerAngles = {0, 0, 0};
    vec3 targetLinearVelocity = {0, 0, 0};
    vec3 targetAngularVelocity = {0, 0, 0};
};

struct TargetLegTrajectory
{
    mat34 p;
    mat34 v;
    mat34 a;
    mat34 toeLocation;
};

struct TargetJointTrajectory
{
    mat34 targetJointPosition;
    mat34 targetJointVelocity;
    mat34 targetJointTorque;
};
}  // namespace robot_target_trajectory

namespace robot_user_cmd
{
struct UserCmd
{
    vec3 veloCmd = {0, 0, 0};
    vec3 angveloCmd = {0, 0, 0};
    int gaitCmd = 0;
    bool isKeyPressed = false;
};
}  // namespace robot_user_cmd

}  // namespace Galileo