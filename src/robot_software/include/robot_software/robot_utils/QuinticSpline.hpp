#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace Galileo
{
class QuinticTrajectory
{
public:
    // 构造函数，设置起点、终点和中间点的约束
    QuinticTrajectory(double t0,
                      double p0,
                      double v0,
                      double a0,
                      double tm,
                      double pm,
                      double tf,
                      double pf,
                      double vf,
                      double af)
    {
        // 时间点
        t0_ = t0;
        tf_ = tf;
        tm_ = tm;

        // 构造线性方程组
        Eigen::MatrixXd A(7, 6);  // 7个条件，6个未知数
        Eigen::VectorXd b(7);

        // 起点条件
        A.row(0) << 1, t0, t0 * t0, t0 * t0 * t0, t0 * t0 * t0 * t0, t0 * t0 * t0 * t0 * t0;
        b(0) = p0;

        A.row(1) << 0, 1, 2 * t0, 3 * t0 * t0, 4 * t0 * t0 * t0, 5 * t0 * t0 * t0 * t0;
        b(1) = v0;

        A.row(2) << 0, 0, 2, 6 * t0, 12 * t0 * t0, 20 * t0 * t0 * t0;
        b(2) = a0;

        // 终点条件
        A.row(3) << 1, tf, tf * tf, tf * tf * tf, tf * tf * tf * tf, tf * tf * tf * tf * tf;
        b(3) = pf;

        A.row(4) << 0, 1, 2 * tf, 3 * tf * tf, 4 * tf * tf * tf, 5 * tf * tf * tf * tf;
        b(4) = vf;

        A.row(5) << 0, 0, 2, 6 * tf, 12 * tf * tf, 20 * tf * tf * tf;
        b(5) = af;

        // 中间点位置约束
        A.row(6) << 1, tm, tm * tm, tm * tm * tm, tm * tm * tm * tm, tm * tm * tm * tm * tm;
        b(6) = pm;

        // 求解系数
        Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
        a_ = coeffs;
    }

    // 获取给定时间点的位置
    double getPosition(double t) const
    {
        return a_(0) + a_(1) * t + a_(2) * t * t + a_(3) * t * t * t + a_(4) * t * t * t * t
               + a_(5) * t * t * t * t * t;
    }

    // 获取给定时间点的速度
    double getVelocity(double t) const
    {
        return a_(1) + 2 * a_(2) * t + 3 * a_(3) * t * t + 4 * a_(4) * t * t * t
               + 5 * a_(5) * t * t * t * t;
    }

    // 获取给定时间点的加速度
    double getAcceleration(double t) const
    {
        return 2 * a_(2) + 6 * a_(3) * t + 12 * a_(4) * t * t + 20 * a_(5) * t * t * t;
    }

private:
    double t0_, tf_, tm_;  // 时间点
    Eigen::VectorXd a_;    // 多项式系数
};
}  // namespace Galileo