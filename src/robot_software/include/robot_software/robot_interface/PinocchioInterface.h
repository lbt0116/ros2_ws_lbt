//
// Created by lbt on 24-12-9.
//

#ifndef PINOCCHIOINTERFACEIMPL_H
#define PINOCCHIOINTERFACEIMPL_H
#include <memory>
#include <custom_msgs/msg/detail/mujoco_msg__struct.hpp>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "robot_software/robot_utils/MatrixTypes.h"
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/robot_state_msg.hpp"
#include "custom_msgs/msg/mujoco_msg.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace Galileo
{
    class PinocchioInterface
    {
    public:
        PinocchioInterface();
        ~PinocchioInterface(); // 添加析构函数声明

        mat43 get();

        void set_imu_msg(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) const;
        void set_joint_msg(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) const;
        void set_mujoco_msg(const custom_msgs::msg::MujocoMsg::ConstSharedPtr& msg);


        // Eigen::MatrixXd get_centroidal_matrix() const;
        //
        // Eigen::Matrix3d get_inertia_matrix() const;
        //
        // Eigen::MatrixXd get_jb_matrix() const;
        //
        // double get_total_mass() const;

        Eigen::MatrixXd get_jacobian_matrix(const int i) const;

        void update();
        // ~PinocchioInterface() = default;


        //Joint State
        mat34 jonitPos;
        mat34 jonitVelo;

        //Leg State
        mat34 legPosHipInBody;
        mat34 legPosHipInWorld;
        mat34 legPosBaseInBody;
        mat34 legPosBaseInWorld;
        mat34 legVeloInBody;
        mat34 legVeloInWorld;
        mat34 legForceInBody;
        mat34 legForceInWorld;

        //Base State
        mat33 baseRotationMatrix;
        mat33 baseInertiaMatrix;
        mat66 baseSpatialInertiaMatrix;

        vec3 baseAcc;
        vec3 baseAngVelo;

        double totalMass;

    private:
        class PinocchioInterfaceImpl;
        std::unique_ptr<PinocchioInterfaceImpl> impl_; // 使用PImpl隐藏实现细节
    };
}


#endif //PINOCCHIOINTERFACEIMPL_H
