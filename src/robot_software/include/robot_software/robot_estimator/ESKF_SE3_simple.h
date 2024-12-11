//
// Created by lbt on 24-8-5.
//

#ifndef ESKF_SE3_SIMPLE_H
#define ESKF_SE3_SIMPLE_H

#include "eigen3/Eigen/Dense"
#include <deque>
#include <mutex>

class ESKF_SE3_simple
{
private:
    static constexpr int StateNum = 18;
    static constexpr int NoiseNum = 12;
    static constexpr int ObNum = 15;

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

    Eigen::Matrix<double, 6, 6> Jb_mat;

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

    std::deque<Eigen::MatrixXd> InputSequence;
    std::deque<Eigen::MatrixXd> StateSequence; //rho: [theta r] xi: [w v]
    std::deque<Eigen::MatrixXd> ObsSequence; //rho: [theta r] xi: [w v]
    Eigen::Matrix<double, 12, 1> Obs_Vec;
    Eigen::Matrix<double, 3, 1> a_m;

    std::mutex mutex_;

    ESKF_SE3_simple();

    void setTransMatrix();
    void setMeasureMatrix();
    void setErrorMatrix(const Eigen::Matrix<double, 3, 1>& am, const Eigen::Matrix<double, 3, 1>& wm,
                        const Eigen::Matrix<double, 3, 1>& vm,
                        const Eigen::Matrix<double, 3, 3>& Rm,
                        const Eigen::Matrix<double, 3, 1>& uc);


    void run();
};


#endif //ESKF_SE3_SIMPLE_H
