//
// Created by lbt on 24-12-9.
//
#include "robot_software/robot_interface/PinocchioInterface.h"
#include <pinocchio/fwd.hpp>

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "robot_software/robot_utils/UtilFunc.h"
#include "pinocchio/algorithm/centroidal.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>


using namespace Eigen;
// using namespace pinocchio;
namespace pin = pinocchio;

namespace Galileo
{
    class PinocchioInterface::PinocchioInterfaceImpl
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


        void update_pinocchio(PinocchioInterface* pin_);

        pinocchio::Model model; // Pinocchio的Model
        pinocchio::Data data; // Pinocchio的Data
    };

    PinocchioInterface::PinocchioInterfaceImpl::PinocchioInterfaceImpl()
    {
        const std::string urdf_filename = std::string(
            "/home/lbt/ros2_ws/src/mujoco_node/models/quadruped/urdf/quadruped.urdf");
        pinocchio::urdf::buildModel(urdf_filename, model);
        data = pinocchio::Data(model);
    }

    void PinocchioInterface::PinocchioInterfaceImpl::update_pinocchio(PinocchioInterface* pin_)
    {
        //pinocchio 四元数 xyzw
        //Eigen wxyz
        //mujoco wxyz


        q.block(0, 0, 3, 1).setZero();
        q.block(3, 0, 4, 1) = imuQuant;
        for (int i = 0; i < 4; i++)
        {
            q.block(7 + 3 * i, 0, 3, 1) = jointPos.col(i);
        }

        dq.block(0, 0, 3, 1).setZero();
        dq.block(3, 0, 3, 1) = imuAngvelo;
        for (int i = 0; i < 4; i++)
        {
            dq.block(6 + 3 * i, 0, 3, 1) = jointVelo.col(i);
        }

        ddq.block(0, 0, 3, 1) = imuAcc; //ddpB
        ddq.block(3, 0, 3, 1).setZero();
        for (int i = 0; i < 4; i++)
        {
            ddq.block(6 + 3 * i, 0, 3, 1).setZero();
        }

        // LegPhase = Phase;//
        pin::forwardKinematics(model, data, q, dq,
                               ddq);
        pin::framesForwardKinematics(model, data, q);
        pin::computeJointJacobians(model, data, q);
        // 更新数据结构（计算质心相关信息）
        pin::centerOfMass(model, data, q); // 计算质心位置
        pin::computeCentroidalMap(model, data, q); // 计算质心动力学映射


        pin::getFrameJacobian(model, data, 12, pinocchio::LOCAL_WORLD_ALIGNED,
                              Jacobian[0]); //?什么坐标系
        pin::getFrameJacobian(model, data, 20, pinocchio::LOCAL_WORLD_ALIGNED,
                              Jacobian[1]);
        pin::getFrameJacobian(model, data, 28, pinocchio::LOCAL_WORLD_ALIGNED,
                              Jacobian[2]);
        pin::getFrameJacobian(model, data, 36, pinocchio::LOCAL_WORLD_ALIGNED,
                              Jacobian[3]);

        pin::updateFramePlacements(model, data);

        pin_->jonitPos = jointPos;
        pin_->jonitVelo = jointVelo;

        pin_->totalMass = data.mass[0];
        pin_->baseInertiaMatrix = model.inertias[0].inertia().matrix();
        pin_->baseSpatialInertiaMatrix <<
            model.inertias[0].inertia().matrix(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(),
            data.mass[0] * Eigen::Matrix3d::Identity();

        pin_->baseRotationMatrix = data.oMi[1].rotation();

        pin_->baseAcc = imuAcc;
        pin_->baseAngVelo = imuAngvelo;

        for (int leg = 0; leg < 4; ++leg)
        {
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
            pin_->legPosBaseInBody.col(leg) = foot_pos_in_body;

            // 足端相对于机身在世界坐标系中的位置
            pin_->legPosBaseInWorld.col(leg) = foot_pose_in_world.translation();

            // ---- 5. 计算足端相对于髋关节的位置 ----
            // 足端相对于髋关节在世界坐标系中的位置
            Eigen::Vector3d foot_pos_rel_hip_in_world = hip_pose_in_world.actInv(foot_pose_in_world).translation();
            pin_->legPosHipInWorld.col(leg) = foot_pos_rel_hip_in_world;

            // 足端相对于髋关节在机身坐标系中的位置
            Eigen::Vector3d foot_pos_rel_hip_in_body = base_pose_in_world.actInv(hip_pose_in_world).rotation() *
                foot_pos_rel_hip_in_world;
            pin_->legPosHipInBody.col(leg) = foot_pos_rel_hip_in_body;

            // ---- 6. 提取速度信息 ----
            // 足端在世界坐标系中的速度
            pinocchio::Motion foot_vel_in_world = pinocchio::getFrameVelocity(
                model, data, foot_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
            // 髋关节在世界坐标系中的速度
            pinocchio::Motion hip_vel_in_world = pinocchio::getFrameVelocity(
                model, data, hip_frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

            // 足端相对于髋关节的速度（世界坐标系）
            Eigen::Vector3d foot_vel_rel_hip_in_world = foot_vel_in_world.linear() - hip_vel_in_world.linear();
            pin_->legVeloInWorld.col(leg) = foot_vel_rel_hip_in_world;

            // 足端相对于髋关节的速度（机身坐标系）
            Eigen::Vector3d foot_vel_rel_hip_in_body = base_pose_in_world.actInv(hip_pose_in_world).rotation() *
                foot_vel_rel_hip_in_world;
            pin_->legVeloInBody.col(leg) = foot_vel_rel_hip_in_body;
        }
    }


    PinocchioInterface::PinocchioInterface() : impl_(std::make_unique<PinocchioInterfaceImpl>())
    {
    }

    PinocchioInterface::~PinocchioInterface() = default;
    // 当使用unique_ptr管理PImpl类时,需要在头文件中声明析构函数
    // 析构函数的实现必须在cpp文件中,因为此时已经有了完整的PinocchioInterfaceImpl类型定义
    // 这样可以解决不完整类型的问题


    mat43 PinocchioInterface::get()
    {
        auto a = impl_->legPos;
        return a;
    }

    void PinocchioInterface::set_imu_msg(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) const
    {
        // 从IMU消息中提取quanternion
        impl_->imuQuant << msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w;

        // 从IMU消息中提取线性加速度
        impl_->imuAcc << msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z;

        // 从IMU消息中提取角速度
        impl_->imuAngvelo << msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;
    }

    void PinocchioInterface::set_joint_msg(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) const
    {
        // 按顺序将 position 数据赋值到矩阵中
        for (int i = 0; i < 12; ++i)
        {
            const int row = i % 3; // 确定当前数据在矩阵中的行
            const int col = i / 3; // 确定当前数据在矩阵中的列
            impl_->jointPos(row, col) = msg->position[i];
            impl_->jointVelo(row, col) = msg->velocity[i];
            impl_->jointTorque(row, col) = msg->effort[i];
        }
    }

    void PinocchioInterface::set_mujoco_msg(const custom_msgs::msg::MujocoMsg::ConstSharedPtr& msg)
    {
        auto cols = msg->ground_reaction_force.size() / 3;
        stdVectorToEigen(msg->ground_reaction_force, 3, cols);
    }


    Eigen::MatrixXd PinocchioInterface::get_jacobian_matrix(const int i) const
    {
        return impl_->Jacobian[i];
    }


    void PinocchioInterface::update()
    {
        impl_->update_pinocchio(this);
    }
}
