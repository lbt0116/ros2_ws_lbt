#pragma once
#include <Eigen/Dense>
#include <array>

#include "pinocchio/multibody/model.hpp"
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
    vec3 inertiaVector = {0.2, 0.8, 0.9};
    mat33 inertiaMatrix;         // dont use in constructor
    mat66 spatialInertiaMatrix;  // dont use in constructor
    vec3 gravity = {0, 0, -9.81};
};

// 机器人状态相关数据结构
namespace robot_state
{
struct SensorData
{
    mat34 jointPosition;
    mat34 jointVelocity;
    mat34 jointTorque;
    Eigen::Quaterniond imuQuant;
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

struct BaseState  // world frame
{
    vec3 position;
    vec3 positionRelative;          // 相对于基座的位置
    vec3 eulerAngles;               // 欧拉角
    mat33 rotationMatrix;           // 旋转矩阵
    Eigen::Quaterniond quaternion;  // 四元数 (x,y,z,w)
    vec3 linearVelocity;            // 线速度
    vec3 angularVelocity;           // 角速度

    vec3 acceleration;

    vec3 p_real;  // mujoco 真实位置
    vec3 v_real;  // mujoco 真实速度
};

struct ContactState
{
    vec4i isContact;
    mat34 contactForce;
};

// using ModelPtr = std::shared_ptr<pinocchio::Model>;
// std::shared_ptr<pinocchio::Model> modelPtr;

}  // namespace robot_state

namespace robot_FSM
{

struct legState
{
    vec4i legPhase = {1, 1, 1, 1};
    int isStep;
    Eigen::Matrix<double, 4, 1> timeSw;
};

}  // namespace robot_FSM

namespace robot_target_trajectory
{

struct TargetBaseTrajectory
{
    vec3 targetPosition = {0, 0, 0.5};
    Eigen::Quaterniond targetQuaternion = Eigen::Quaterniond::Identity();  // 目标四元数
    vec3 targetLinearVelocity = {0, 0, 0};
    vec3 targetAngularVelocity = {0, 0, 0};
};

struct TargetLegTrajectory
{
    mat34 targetLegPosition = mat34::Zero();  //{0, 0, -0.5, 0, 0, -0.5, 0, 0, -0.5, 0, 0, -0.5}
    mat34 targetLegVelocity = mat34::Zero();
    mat34 targetLegAcceleration = mat34::Zero();
    mat34 targetToeLocation = mat34::Zero();
};

struct TargetJointTrajectory
{
    mat34 targetJointPosition = mat34::Zero();
    mat34 targetJointVelocity = mat34::Zero();
    mat34 targetJointTorque = mat34::Zero();
};
}  // namespace robot_target_trajectory

namespace robot_controller
{
struct BallanceController
{
    mat34 f;
};

struct SwingLegController
{
    mat34 f;
};

struct JointController
{
    mat34 composedLegForce;
    mat34 tau;
};

struct GeoController
{
    mat34 f;
};
}  // namespace robot_controller

namespace robot_user_cmd
{
struct UserCmd
{
    vec3 veloCmd = {0, 0, 0};
    vec3 angveloCmd = {0, 0, 0};
    vec3 angCmd = {0, 0, 0};
    int gaitCmd = 0;
    int ctrlType = 0;
    bool isKeyPressed = false;
};
}  // namespace robot_user_cmd

}  // namespace Galileo