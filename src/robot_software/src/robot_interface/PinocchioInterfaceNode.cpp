//
// Created by lbt on 24-12-9.
//
#include "robot_software/robot_interface/PinocchioInterfaceNode.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/fwd.hpp>

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "robot_software/robot_utils/UtilFunc.h"
using namespace Eigen;
// using namespace pinocchio;
namespace pin = pinocchio;

namespace Galileo
{
class PinocchioInterfaceNode::PinocchioInterfaceImpl
{
public:
    PinocchioInterfaceImpl();
    Vector4d imuQuant;
    Vector3d imuAngvelo;
    Vector3d imuAcc;

    mat34 jointPos;
    mat34 jointVelo;
    mat34 jointTorque;
    mat43 legPos;

    Matrix<double, 19, 1> q;
    Matrix<double, 18, 1> dq;
    Matrix<double, 18, 1> ddq;
    Matrix<double, 6, 18> Jacobian[4];

    void update_pinocchio(PinocchioInterfaceNode* pin_);

    pinocchio::Model model;  // Pinocchio的Model
    pinocchio::Data data;    // Pinocchio的Data
};

PinocchioInterfaceNode::PinocchioInterfaceImpl::PinocchioInterfaceImpl()
{
    const std::string urdf_filename =
        std::string("/home/lbt/ros2_ws/src/mujoco_node/models/quadruped/urdf/quadruped.urdf");
    pinocchio::urdf::buildModel(urdf_filename, model);
    data = pinocchio::Data(model);
}

void PinocchioInterfaceNode::PinocchioInterfaceImpl::update_pinocchio(PinocchioInterfaceNode* pin_)
{
    auto sensorData = pin_->dataCenter.read<robot_state::SensorData>();  // read
                                                                         // 返回的是一个智能指针

    pin_->baseState = *pin_->dataCenter.read<robot_state::BaseState>();
    // pinocchio 四元数 xyzw
    // tf2 四元数 xyzw
    // Eigen wxyz
    // mujoco wxyz
    tf2::Quaternion quat(
        sensorData->imuQuant.x(), sensorData->imuQuant.y(), sensorData->imuQuant.z(), sensorData->imuQuant.w());
    // 将四元数转换为欧拉角 (Roll, Pitch, Yaw)
    Eigen::Vector3d eulerAngles;
    tf2::Matrix3x3(quat).getRPY(eulerAngles(0), eulerAngles(1), eulerAngles(2));

    quat.setRPY(eulerAngles(0), eulerAngles(1), 0);

    q.block(0, 0, 3, 1).setZero();
    
    // q(3) = quat.x();
    // q(4) = quat.y();
    // q(5) = quat.z();
    // q(6) = quat.w();

    q(3) = sensorData->imuQuant.x();
    q(4) = sensorData->imuQuant.y();
    q(5) = sensorData->imuQuant.z();
    q(6) = sensorData->imuQuant.w();

    for (int i = 0; i < 4; i++)
    {
        q.block(7 + 3 * i, 0, 3, 1) = sensorData->jointPosition.col(i);
    }

    dq.block(0, 0, 3, 1).setZero();
    dq.block(3, 0, 3, 1) = sensorData->imuAngvelo;
    for (int i = 0; i < 4; i++)
    {
        dq.block(6 + 3 * i, 0, 3, 1) = sensorData->jointVelocity.col(i);
    }

    ddq.block(0, 0, 3, 1) = sensorData->imuAcc;  // ddpB
    ddq.block(3, 0, 3, 1).setZero();
    for (int i = 0; i < 4; i++)
    {
        ddq.block(6 + 3 * i, 0, 3, 1).setZero();
    }

    // LegPhase = Phase;//
    pin::forwardKinematics(model, data, q, dq, ddq);
    pin::framesForwardKinematics(model, data, q);
    pin::computeJointJacobians(model, data, q);
    // 更新数据结构（计算质心相关信息）
    pin::centerOfMass(model, data, q);          // 计算质心位置
    pin::computeCentroidalMap(model, data, q);  // 计算质心动力学映射

    pin::updateFramePlacements(model, data);
    pin::getFrameJacobian(model, data, 12, pinocchio::LOCAL_WORLD_ALIGNED, Jacobian[0]);
    pin::getFrameJacobian(model, data, 20, pinocchio::LOCAL_WORLD_ALIGNED, Jacobian[1]);
    pin::getFrameJacobian(model, data, 28, pinocchio::LOCAL_WORLD_ALIGNED, Jacobian[2]);
    pin::getFrameJacobian(model, data, 36, pinocchio::LOCAL_WORLD_ALIGNED, Jacobian[3]);

    pin_->robotConstants.mass = data.mass[0];
    pin_->robotConstants.inertiaMatrix = model.inertias[0].inertia().matrix();
    pin_->robotConstants.spatialInertiaMatrix << model.inertias[0].inertia().matrix(), Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Zero(), data.mass[0] * Eigen::Matrix3d::Identity();

    pin_->jointState.jointPosition = sensorData->jointPosition;
    pin_->jointState.jointVelocity = sensorData->jointVelocity;
    pin_->jointState.jointTorque = sensorData->jointTorque;

    pin_->baseState.quaternion = sensorData->imuQuant;
    pin_->baseState.rotationMatrix = sensorData->imuQuant.toRotationMatrix();
    // pin_->baseState.rotationMatrix = data.oMf[model.getFrameId("base_link")].rotation();

    pin_->baseState.eulerAngles = eulerAngles;

    pin_->baseState.acceleration = data.oMf[model.getFrameId("base_link")].rotation() * sensorData->imuAcc;
    pin_->baseState.angularVelocity = data.oMf[model.getFrameId("base_link")].rotation() * sensorData->imuAngvelo;

    for (int leg = 0; leg < 4; ++leg)
    {
        //  ---- 1. 足端雅克比矩阵 ----
        pin_->legState.legJacobian[leg] = Jacobian[leg];
        // ---- 2. 足端的 frame ID ----
        std::string foot_frame_name = "toe" + std::to_string(leg + 1) + "Link";
        pinocchio::FrameIndex foot_frame_id = model.getFrameId(foot_frame_name);

        // 获取髋关节 frame ID
        std::string hip_frame_name = "Hip" + std::to_string(leg + 1) + "Link";
        pinocchio::FrameIndex hip_frame_id = model.getFrameId(hip_frame_name);

        // ---- 3. 提取足端和髋关节的位姿 ----
        // 足端在世界坐标系中的位姿
        const pinocchio::SE3& foot_pose_in_world = data.oMf[foot_frame_id];
        // 髋关节在世界坐标系中的位姿
        const pinocchio::SE3& hip_pose_in_world = data.oMf[hip_frame_id];
        // 机身在世界坐标系中的位姿
        const pinocchio::SE3& base_pose_in_world = data.oMf[model.getFrameId("base_link")];

        // ---- 4. 计算足端相对于机身的直接位置 ----
        // 足端相对于机身在机身坐标系中的位置
        Eigen::Vector3d foot_pos_in_body = base_pose_in_world.actInv(foot_pose_in_world).translation();
        pin_->legState.legPosBaseInBody.col(leg) = foot_pos_in_body;

        // 足端相对于机身在世界坐标系中的位置
        pin_->legState.legPosBaseInWorld.col(leg) = foot_pose_in_world.translation();

        // ---- 5. 计算足端相对于髋关节的位置 ----
        // 足端相对于髋关节在世界坐标系中的位置
        pin_->legState.legPosHipInWorld.col(leg) = foot_pose_in_world.translation() - hip_pose_in_world.translation();

        // 足端相对于髋关节在机身坐标系中的位置
        Eigen::Vector3d foot_pos_rel_hip_in_body =
            base_pose_in_world.actInv(hip_pose_in_world).rotation() * pin_->legState.legPosHipInWorld.col(leg);
        pin_->legState.legPosHipInBody.col(leg) = foot_pos_rel_hip_in_body;

        // ---- 6. 提取速度信息 ----
        // 足端在世界坐标系中的速度
        pinocchio::Motion foot_vel_in_world =
            pinocchio::getFrameVelocity(model, data, foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
        // 髋关节在世界坐标系中的速度
        pinocchio::Motion hip_vel_in_world =
            pinocchio::getFrameVelocity(model, data, hip_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

        // 足端相对于髋关节的速度（世界坐标系）
        Eigen::Vector3d foot_vel_rel_hip_in_world = foot_vel_in_world.linear() - hip_vel_in_world.linear();
        pin_->legState.legVeloInWorld.col(leg) = foot_vel_rel_hip_in_world;

        // 足端相对于髋关节的速度（机身坐标系）
        Eigen::Vector3d foot_vel_rel_hip_in_body =
            base_pose_in_world.actInv(hip_pose_in_world).rotation() * foot_vel_rel_hip_in_world;
        pin_->legState.legVeloInBody.col(leg) = foot_vel_rel_hip_in_body;
    }

    pin_->dataCenter.write(pin_->robotConstants);
    pin_->dataCenter.write(pin_->jointState);
    pin_->dataCenter.write(pin_->baseState);
    pin_->dataCenter.write(pin_->legState);
}

PinocchioInterfaceNode::PinocchioInterfaceNode()
    : Node("pinocchio_interface", rclcpp::NodeOptions().use_intra_process_comms(true)),
      impl_(std::make_unique<PinocchioInterfaceImpl>()),
      dataCenter(DataCenter::getInstance())
{
    triggerSub_ = this->create_subscription<std_msgs::msg::Bool>(
        "trigger",
        rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data),
        std::bind(&PinocchioInterfaceNode::trigger_callback, this, std::placeholders::_1));
    dataCenter.write(impl_->model);
    RCLCPP_INFO(this->get_logger(), "PinocchioInterfaceNode initialized");
}

PinocchioInterfaceNode::~PinocchioInterfaceNode() = default;
// 当使用unique_ptr管理PImpl类时,需要在头文件中声明析构函数
// 析构函数的实现必须在cpp文件中,因为此时已经有了完整的PinocchioInterfaceImpl类型定义
// 这样可以解决不完整类型的问题

void PinocchioInterfaceNode::trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    // RCLCPP_INFO(this->get_logger(), "Trigger received: %d", msg->data);
    impl_->update_pinocchio(this);
}

}  // namespace Galileo
