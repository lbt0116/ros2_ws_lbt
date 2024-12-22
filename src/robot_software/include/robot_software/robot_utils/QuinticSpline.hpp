#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace Galileo
{
class QuinticTrajectory
{
public:
    QuinticTrajectory(double p0,
                      double v0,
                      double a0,  // 起点的位置、速度、加速度
                      double tm,
                      double pm,  // 中间点的位置和时间
                      double pf,
                      double vf,
                      double af,  // 终点的位置、速度、加速度
                      double T)   // 轨迹总时间
    {
        // 初始化成员变量
        this->T_ = T;
        this->Tm_ = tm;

        // 计算七次多项式系数
        computeCoefficients(p0, v0, a0, pf, vf, af, pm, tm, T);
    }

    // 计算给定时间点的轨迹：位置、速度、加速度
    void evaluate(double t, double &position, double &velocity, double &acceleration) const
    {
        // 使用七次多项式计算位置
        position = coeffs_[0] + coeffs_[1] * t + coeffs_[2] * t * t + coeffs_[3] * t * t * t
                   + coeffs_[4] * t * t * t * t + coeffs_[5] * t * t * t * t * t
                   + coeffs_[6] * t * t * t * t * t * t + coeffs_[7] * t * t * t * t * t * t * t;

        // 计算速度
        velocity = coeffs_[1] + 2 * coeffs_[2] * t + 3 * coeffs_[3] * t * t
                   + 4 * coeffs_[4] * t * t * t + 5 * coeffs_[5] * t * t * t * t
                   + 6 * coeffs_[6] * t * t * t * t * t + 7 * coeffs_[7] * t * t * t * t * t * t;

        // 计算加速度
        acceleration = 2 * coeffs_[2] + 6 * coeffs_[3] * t + 12 * coeffs_[4] * t * t
                       + 20 * coeffs_[5] * t * t * t + 30 * coeffs_[6] * t * t * t * t
                       + 42 * coeffs_[7] * t * t * t * t * t;
    }

private:
    double T_;                      // 总时间
    double Tm_;                     // 中间点的时间
    std::array<double, 8> coeffs_;  // 七次多项式系数

    // 计算七次多项式的系数
    void computeCoefficients(double p0,
                             double v0,
                             double a0,  // 起点的位置、速度、加速度
                             double pf,
                             double vf,
                             double af,  // 终点的位置、速度、加速度
                             double pm,
                             double tm,
                             double T)  // 中间点的位置、时间和轨迹总时间
    {
        Eigen::Matrix<double, 8, 8> A;
        Eigen::Vector<double, 8> b;

        // 起点条件
        A(0, 0) = 1;
        A(0, 1) = 0;
        A(0, 2) = 0;
        A(0, 3) = 0;
        A(0, 4) = 0;
        A(0, 5) = 0;
        A(0, 6) = 0;
        A(0, 7) = 0;
        b(0) = p0;  // p(0) = p0
        A(1, 0) = 0;
        A(1, 1) = 1;
        A(1, 2) = 0;
        A(1, 3) = 0;
        A(1, 4) = 0;
        A(1, 5) = 0;
        A(1, 6) = 0;
        A(1, 7) = 0;
        b(1) = v0;  // p'(0) = v0
        A(2, 0) = 0;
        A(2, 1) = 0;
        A(2, 2) = 2;
        A(2, 3) = 0;
        A(2, 4) = 0;
        A(2, 5) = 0;
        A(2, 6) = 0;
        A(2, 7) = 0;
        b(2) = a0;  // p''(0) = a0

        // 终点条件
        double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T, T6 = T5 * T, T7 = T6 * T;
        A(3, 0) = 1;
        A(3, 1) = T;
        A(3, 2) = T2;
        A(3, 3) = T3;
        A(3, 4) = T4;
        A(3, 5) = T5;
        A(3, 6) = T6;
        A(3, 7) = T7;
        b(3) = pf;  // p(T) = pf
        A(4, 0) = 0;
        A(4, 1) = 1;
        A(4, 2) = 2 * T;
        A(4, 3) = 3 * T2;
        A(4, 4) = 4 * T3;
        A(4, 5) = 5 * T4;
        A(4, 6) = 6 * T5;
        A(4, 7) = 7 * T6;
        b(4) = vf;  // p'(T) = vf
        A(5, 0) = 0;
        A(5, 1) = 0;
        A(5, 2) = 2;
        A(5, 3) = 6 * T;
        A(5, 4) = 12 * T2;
        A(5, 5) = 20 * T3;
        A(5, 6) = 30 * T4;
        A(5, 7) = 42 * T5;
        b(5) = af;  // p''(T) = af

        // 中间点条件
        double Tm2 = tm * tm, Tm3 = Tm2 * tm, Tm4 = Tm3 * tm, Tm5 = Tm4 * tm, Tm6 = Tm5 * tm,
               Tm7 = Tm6 * tm;
        A(6, 0) = 1;
        A(6, 1) = tm;
        A(6, 2) = Tm2;
        A(6, 3) = Tm3;
        A(6, 4) = Tm4;
        A(6, 5) = Tm5;
        A(6, 6) = Tm6;
        A(6, 7) = Tm7;
        b(6) = pm;  // p(Tm) = pm

        // 假设中间点速度为 0（可根据需求修改）
        A(7, 0) = 0;
        A(7, 1) = 1;
        A(7, 2) = 2 * tm;
        A(7, 3) = 3 * Tm2;
        A(7, 4) = 4 * Tm3;
        A(7, 5) = 5 * Tm4;
        A(7, 6) = 6 * Tm5;
        A(7, 7) = 7 * Tm6;
        b(7) = 0;

        // 解方程组
        Eigen::Vector<double, 8> x = A.fullPivLu().solve(b);

        // 转换为 std::array
        for (int i = 0; i < 8; ++i)
        {
            coeffs_[i] = x(i);
        }
    }
};
}  // namespace Galileo