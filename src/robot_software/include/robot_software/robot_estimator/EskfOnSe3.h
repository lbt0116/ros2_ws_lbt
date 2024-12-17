//
// Created by lbt on 24-12-11.
//

#ifndef ESKFONSE3_H
#define ESKFONSE3_H
#include "eigen3/Eigen/Dense"
#include "robot_software/robot_utils/DataCenter.hpp"
#include "robot_software/robot_utils/DataTypes.hpp"

namespace Galileo
{
class EskfOnSe3
{
private:
    static constexpr int StateNum = 18;
    static constexpr int NoiseNum = 12;
    static constexpr int ObNum = 15;

    const Eigen::Vector3d grav = {0, 0, -9.8};

    Eigen::Matrix<double, StateNum, 1> dx_mat;
    Eigen::Matrix<double, StateNum, 1> dxpre_mat;

    Eigen::Matrix<double, StateNum, StateNum> F_mat;
    Eigen::Matrix<double, StateNum, StateNum> Fk_mat;
    Eigen::Matrix<double, StateNum, NoiseNum> Fn_mat;

    Eigen::Matrix<double, StateNum, StateNum> P_mat;
    Eigen::Matrix<double, StateNum, StateNum> Ppre_mat;

    Eigen::Matrix<double, StateNum, ObNum> K_mat;
    Eigen::Matrix<double, ObNum, StateNum> H_mat;

    Eigen::Matrix<double, ObNum, 1> Error_Vec;

    // 从pin_中缓存的数据
    Eigen::Matrix3d baseInertiaMatrix_;
    double totalMass_;
    Eigen::Matrix<double, 6, 6> baseSpatialInertiaMatrix_;
    Eigen::Matrix<double, 3, 4> legVeloInWorld_;
    Eigen::Matrix<double, 3, 4> legForceInWorld_;
    Eigen::Vector3d baseAcc_;
    Eigen::Vector3d baseAngVelo_;
    Eigen::Matrix3d baseRotationMatrix_;

    // 添加更新函数声明
    void updateStates(const Eigen::Matrix<int, 4, 1>& phase);
    DataCenter& dataCenter_;

public:
    Eigen::Matrix<double, StateNum, 1> Q_vec;
    Eigen::Matrix<double, NoiseNum, 1> Qn_vec;
    Eigen::Matrix<double, ObNum, 1> R_vec;
    Eigen::Matrix<double, StateNum, StateNum> Q_mat;
    Eigen::Matrix<double, NoiseNum, NoiseNum> Qn_mat;
    Eigen::Matrix<double, ObNum, ObNum> R_mat;

    Eigen::Matrix<double, 3, 1> p;
    Eigen::Matrix<double, 3, 1> v;
    Eigen::Matrix<double, 3, 1> q;
    Eigen::Matrix<double, 3, 3> R;
    Eigen::Matrix<double, 3, 1> w;
    Eigen::Matrix<double, 3, 1> Fe;
    Eigen::Matrix<double, 3, 1> Me;

    Eigen::Matrix<double, 4, 4> g;
    Eigen::Matrix<double, 6, 1> xi;
    Eigen::Matrix<double, 6, 1> ue;

    EskfOnSe3();

    void setTransMatrix();
    void setMeasureMatrix();
    void setErrorMatrix();
    Eigen::Matrix<double, 3, 1> uc_;
    Eigen::Matrix<double, 3, 1> vm_;

    void run(const Eigen::Matrix<int, 4, 1>& phase);
};
}  // namespace Galileo

#endif  // ESKFONSE3_H
