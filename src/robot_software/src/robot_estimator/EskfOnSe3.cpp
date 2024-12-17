//
// Created by lbt on 24-12-11.
//

#include "robot_software/robot_estimator/EskfOnSe3.h"

#include "robot_software/robot_utils/UtilFunc.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

namespace Galileo
{
EskfOnSe3::EskfOnSe3()
    : dataCenter_(DataCenter::getInstance())
{
    F_mat.setZero();
    Fk_mat.setZero();
    Fn_mat.setZero();

    H_mat.setZero();
    K_mat.setZero();
    dxpre_mat.setZero();
    P_mat.setIdentity();
    Ppre_mat.setIdentity();

    g.setIdentity();
    xi.setZero();
    ue.setZero();

    p.setZero();
    v.setZero();
    R.setIdentity();
    w.setZero();

    Fe.setZero();
    Me.setZero();
    // todo para
}

void EskfOnSe3::updateStates(const Eigen::Matrix<int, 4, 1>& phase)
{
    auto baseState = dataCenter_.read<robot_state::BaseState>();
    auto jointState = dataCenter_.read<robot_state::JointState>();
    auto robotConstants = dataCenter_.read<robot_constants>();
    auto legState = dataCenter_.read<robot_state::LegState>();

    totalMass_ = robotConstants->mass;

    baseInertiaMatrix_ = robotConstants->inertiaMatrix;
    baseSpatialInertiaMatrix_ = robotConstants->spatialInertiaMatrix;

    baseAcc_ = baseState->acceleration;
    baseAngVelo_ = baseState->angularVelocity;
    baseRotationMatrix_ = baseState->rotationMatrix;

    legVeloInWorld_ = legState->legVeloInWorld;
    legForceInWorld_ = legState->legForceInWorld;

    vm_ = -legVeloInWorld_ * phase.cast<double>() / phase.sum();
    uc_ = legForceInWorld_ * phase.cast<double>();
}

void EskfOnSe3::setTransMatrix()
{
    Eigen::Matrix<double, 6, 6> C_mat;
    C_mat.topLeftCorner<3, 3>() = Sophus::SO3d::hat(baseInertiaMatrix_ * w);
    C_mat.topRightCorner<3, 3>() = totalMass_ * Sophus::SO3d::hat(v);
    C_mat.bottomLeftCorner<3, 3>() = totalMass_ * Sophus::SO3d::hat(v);
    C_mat.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Zero();

    const auto invJb = baseSpatialInertiaMatrix_.inverse();

    F_mat.block(0, 0, 6, 6) = -UtilFnc::adjoint_se3(xi);
    F_mat.block(0, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
    F_mat.block(6, 6, 6, 6) =
        invJb * (C_mat - (-UtilFnc::adjoint_se3(xi).transpose()) * baseSpatialInertiaMatrix_);
    F_mat.block(6, 12, 6, 6) = invJb;

    Fk_mat = F_mat * 0.001 + Eigen::Matrix<double, StateNum, StateNum>::Identity();
    Fn_mat.block<12, 12>(6, 0).setIdentity();
    Fn_mat.block<6, 6>(6, 0) = baseSpatialInertiaMatrix_.inverse();
}

void EskfOnSe3::setMeasureMatrix()
{
    H_mat.block(0, 0, 6, 6).setIdentity();
    H_mat.block(6, 15, 3, 3) = Eigen::Matrix3d::Identity() / totalMass_;
    H_mat.block(9, 6, 6, 6).setIdentity();
}

void EskfOnSe3::setErrorMatrix()
{
    static Eigen::Vector3d pm = Eigen::Vector3d::Zero();
    pm = pm + vm_ * 0.001;
    const Sophus::SE3d g(R, p);
    const Sophus::SE3d gm(baseRotationMatrix_, pm);

    Error_Vec.block(0, 0, 3, 1) = (g.inverse() * gm).log().tail<3>();
    Error_Vec.block(3, 0, 3, 1) = (g.inverse() * gm).log().head<3>();
    Error_Vec.block(6, 0, 3, 1) = baseAcc_ - (uc_ + Fe) / totalMass_ - grav;
    Error_Vec.block(9, 0, 3, 1) = baseAngVelo_ - w;
    Error_Vec.block(12, 0, 3, 1) = vm_ - v;
}

void EskfOnSe3::run(const Eigen::Matrix<int, 4, 1>& phase)
{
    updateStates(phase);

    setTransMatrix();
    setMeasureMatrix();
    setErrorMatrix();

    dx_mat += Fk_mat * dxpre_mat;
    // std::cout << "F_mat:  \n" << F_mat << std::endl << std::endl;

    P_mat.triangularView<Eigen::Upper>() = Fk_mat * Ppre_mat * Fk_mat.transpose();
    P_mat.triangularView<Eigen::Lower>() = P_mat.triangularView<Eigen::Upper>().transpose();

    P_mat.diagonal() = P_mat.diagonal() + Q_mat.diagonal();
    // P_mat = P_mat + Fn_mat * Qn_mat * Fn_mat.transpose();
    // std::cout << P_mat << "   P_mat" << std::endl << std::endl;

    auto diagP = (H_mat * P_mat * H_mat.transpose() + R_mat).inverse();
    // std::cout << diagP << "   diagP" << std::endl << std::endl;

    K_mat = P_mat * H_mat.transpose() * diagP;

    // std::cout << "K_mat   \n" << K_mat << std::endl << std::endl;
    // std::cout << "Q_mat   " << Q_mat << std::endl << std::endl;
    // std::cout << "R_mat   " << R_mat << std::endl << std::endl;
    // std::cout << "H_mat   \n" << H_mat << std::endl << std::endl;

    dxpre_mat = K_mat * Error_Vec;

    Ppre_mat = (Eigen::MatrixXd::Identity(StateNum, StateNum) - K_mat * H_mat) * P_mat;

    Eigen::Vector<double, 6> x_sophus = Eigen::Vector<double, 6>::Zero();
    x_sophus.head<3>() = dxpre_mat.block(3, 0, 3, 1);
    x_sophus.tail<3>() = dxpre_mat.block(0, 0, 3, 1);
    // std::cout << "x_sophus:  " << x_sophus << std::endl << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));

    g = g.eval() * Sophus::SE3d::exp(x_sophus).matrix();
    xi = xi + dxpre_mat.block(6, 0, 6, 1);
    ue = ue + dxpre_mat.block(12, 0, 6, 1);
    // bias.setZero();
    // ue.setZero();
    // std::cout << "x_sophus:  " << x_sophus << std::endl << std::endl;

    R = Sophus::SE3d(g).rotationMatrix();
    q = UtilFnc::normalizeAngles(R.eulerAngles(2, 1, 0));

    p = Sophus::SE3d(g).translation();
    w = xi.head<3>();
    v = xi.tail<3>();
    Me = ue.head<3>();
    Fe = ue.tail<3>();
    // std::cout << "dx_mat  " << dx_mat.transpose() << std::endl << std::endl;

    // std::cout << "Error_Vec:  " << Error_Vec.transpose() << std::endl << std::endl;

    // std::cout << "Fe:  " << Fe.transpose() << std::endl << std::endl;
}
}  // namespace Galileo
