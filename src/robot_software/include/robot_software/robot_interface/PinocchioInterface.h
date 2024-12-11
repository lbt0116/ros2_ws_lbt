//
// Created by lbt on 24-12-9.
//

#ifndef PINOCCHIOINTERFACEIMPL_H
#define PINOCCHIOINTERFACEIMPL_H
#include <memory>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "robot_software/robot_utils/MatrixTypes.h"
#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/robot_state_msg.hpp"
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


        Eigen::MatrixXd get_centroidal_matrix() const;

        Eigen::Matrix3d get_inertia_matrix() const;

        Eigen::MatrixXd get_jb_matrix() const;

        double get_total_mass() const;

        Eigen::MatrixXd get_jacobian_matrix(const int i) const;

        void update();
        // ~PinocchioInterface() = default;
        //State In World Frame
        Eigen::Matrix<double, 3, 3> rotationMatrix;
        //Leg State
        Eigen::Matrix<double, 3, 4> legPosHipInBody;
        Eigen::Matrix<double, 3, 4> legPosHipInWorld;
        Eigen::Matrix<double, 3, 4> legPosBaseInBody;
        Eigen::Matrix<double, 3, 4> legPosBaseInWorld;
        Eigen::Matrix<double, 3, 4> legVeloInBody;
        Eigen::Matrix<double, 3, 4> legVeloInWorld;
        Eigen::Matrix<double, 3, 4> legForceInBody;
        Eigen::Matrix<double, 3, 4> legForceInWorld;

    private:
        class PinocchioInterfaceImpl;
        std::unique_ptr<PinocchioInterfaceImpl> impl_; // 使用PImpl隐藏实现细节
    };
}


#endif //PINOCCHIOINTERFACEIMPL_H
