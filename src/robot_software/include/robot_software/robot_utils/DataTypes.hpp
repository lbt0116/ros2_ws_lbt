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

// 估计器相关数据结构
namespace robot_estimator
{
struct LinearKalmanFilter
{
    vec3 position;
    vec3 eulerAngles;
    vec3 linearVelocity;
    vec3 angularVelocity;
};

struct SensorData
{
    vec3 acceleration;
    vec3 angular_velocity;
    vec3 magnetic_field;
    double timestamp{0.0};
};
}  // namespace robot_estimator

namespace robot_FSM
{
struct GaitSchedule
{
    Eigen::Matrix<int, Eigen::Dynamic, 4> gaitSequence;
    double swingHeight;
    double stand_T;
    double swing_T;
    int gaitTag;
    double standHeight;
};

struct Command
{
    int gaitCmd;
    Eigen::Matrix<int, 4, 1> legPhase;
};
}  // namespace robot_FSM

// 接口相关数据结构
namespace robot_interface
{
struct PinocchioData
{
    mat66 mass_matrix;
    vec6 gravity_terms;
    mat34 jacobian;
};

struct MujocoData
{
    vec12 qpos;
    vec12 qvel;
    vec12 ctrl;
    double time{0.0};
};
}  // namespace robot_interface

// 控制器相关数据结构
namespace robot_controller
{
struct ControlCommand
{
    Eigen::VectorXd desired_position;
    Eigen::VectorXd desired_velocity;
    Eigen::VectorXd kp;
    Eigen::VectorXd kd;
};

struct ControlOutput
{
    Eigen::VectorXd torque;
    double timestamp{0.0};
};
}  // namespace robot_controller

}  // namespace Galileo