//
// Created by lbt on 24-12-9.
//

#ifndef UTILFUNC_H
#define UTILFUNC_H

#include <Eigen/Dense>
#include <vector>
#include <type_traits>

#include <std_msgs/msg/float64_multi_array.hpp>

template <typename Derived>
void eigenToFloat64MultiArray(
    const Eigen::MatrixBase<Derived>& matrix,
    std_msgs::msg::Float64MultiArray& msg)
{
    // 设置维度
    msg.layout.dim.resize(2);
    msg.layout.dim[0].size = matrix.rows();
    msg.layout.dim[1].size = matrix.cols();
    msg.layout.dim[0].stride = matrix.rows() * matrix.cols();
    msg.layout.dim[1].stride = matrix.cols();

    // 预分配数据空间
    msg.data.resize(matrix.size());

    // 使用Map直接映射内存
    Eigen::Map<Eigen::Matrix<double, Derived::RowsAtCompileTime,
                             Derived::ColsAtCompileTime>>(msg.data.data(), matrix.rows(), matrix.cols()) = matrix;
}

// 反向转换函数
template <typename Derived>
void float64MultiArrayToEigen(
    const std_msgs::msg::Float64MultiArray& msg,
    Eigen::MatrixBase<Derived>& matrix_out)
{
    matrix_out.derived() =
        Eigen::Map<const Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>>
        (msg.data.data(), msg.layout.dim[0].size, msg.layout.dim[1].size);
}

// 通用模板函数：将Eigen矩阵转换为std::vector
template <typename Derived>
std::vector<typename Derived::Scalar> eigenToStdVector(const Eigen::MatrixBase<Derived>& matrix)
{
    // 确保输入是Eigen矩阵或向量
    static_assert(std::is_base_of<Eigen::MatrixBase<Derived>, Derived>::value,
                  "Input must be an Eigen matrix or vector");

    // 使用Eigen提供的.data()方法，将矩阵数据以列优先顺序复制到std::vector
    return std::vector<typename Derived::Scalar>(matrix.data(), matrix.data() + matrix.size());
}


#include "eigen3/Eigen/Dense"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

//Sophus SE3 vee hat 向量顺序是先平移后旋转
class UtilFnc
{
public:
    static Eigen::Matrix<double, 6, 1> vee2se3(const Eigen::Matrix<double, 4, 4>& g)
    {
        Eigen::Matrix<double, 6, 1> xi = Eigen::Matrix<double, 6, 1>::Zero();
        xi.head<3>() = Sophus::SO3d::vee(g.topLeftCorner(3, 3));
        xi.tail<3>() = g.topRightCorner(3, 1);
        return xi;
    }

    static Eigen::Matrix<double, 4, 4> hat2SE3(const Eigen::Matrix<double, 6, 1>& xi)
    {
        Eigen::Matrix<double, 4, 4> g = Eigen::Matrix4d::Zero();
        g.topLeftCorner(3, 3) = Sophus::SO3d::hat(xi.head<3>());
        g.topRightCorner(3, 1) = xi.tail<3>();
        return g;
    }

    static Eigen::Matrix<double, 6, 6> adjoint_se3(const Eigen::Matrix<double, 6, 1>& xi)
    {
        // 提取旋转部分和平移部分
        Eigen::Vector3d omega = xi.head<3>(); // 旋转部分
        Eigen::Vector3d v = xi.tail<3>(); // 平移部分

        // 计算\text{ad}_{\xi}矩阵
        Eigen::Matrix<double, 6, 6> ad_xi;
        ad_xi.topLeftCorner<3, 3>() = Sophus::SO3d::hat(omega);
        ad_xi.topRightCorner<3, 3>().setZero();
        ad_xi.bottomLeftCorner<3, 3>() = Sophus::SO3d::hat(v);
        ad_xi.bottomRightCorner<3, 3>() = Sophus::SO3d::hat(omega);

        return ad_xi;
    }

    static Eigen::Matrix<double, 4, 4> Cayley2SE3(const Eigen::Matrix<double, 6, 1>& xi)
    {
        const Eigen::Matrix<double, 4, 4> eye = Eigen::Matrix4d::Identity();

        Eigen::Matrix<double, 4, 4> g = (eye - 0.5 * hat2SE3(xi)).inverse() * (eye + 0.5 *
            hat2SE3(xi));
        // Eigen::Quaterniond q(g.topLeftCorner<3, 3>());
        // g.topLeftCorner<3, 3>() = q.toRotationMatrix(); //保证旋转矩阵正交，但是会损失一点精度

        return g;
    }

    static Eigen::Matrix<double, 6, 1> invCayley2se3(const Eigen::Matrix<double, 4, 4>& g)
    {
        const Eigen::Matrix<double, 4, 4> eye = Eigen::Matrix4d::Identity();

        Eigen::Matrix<double, 6, 1> xi = vee2se3(2 * (g - eye) * (g + eye).inverse());
        return xi; //Rz!=pi
    }


    static Eigen::Vector3d normalizeAngles(const Eigen::Vector3d& angles)
    {
        Eigen::Vector3d normalizedAngles = angles;
        for (int i = 0; i < 3; ++i)
        {
            while (normalizedAngles[i] > M_PI) normalizedAngles[i] -= 2 * M_PI;
            while (normalizedAngles[i] < -M_PI) normalizedAngles[i] += 2 * M_PI;
        }
        return normalizedAngles;
    }

    static Eigen::Vector3d processEulerAngles(const Eigen::Matrix3d& rotationMatrix)
    {
        // 使用局部静态变量来存储上一次的欧拉角状态
        static Eigen::Vector3d previousAngles(0.0, 0.0, 0.0);

        // 将旋转矩阵转换为当前欧拉角 (ZYX顺序)
        Eigen::Vector3d currentAngles = rotationMatrix.eulerAngles(2, 1, 0);

        Eigen::Vector3d closestAngles;
        for (int i = 0; i < 3; ++i)
        {
            double wrappedCurrent = std::fmod(currentAngles[i] + M_PI, 2 * M_PI) - M_PI; // 将角度限制在[-π, π]
            double wrappedPrevious = std::fmod(previousAngles[i] + M_PI, 2 * M_PI) - M_PI;

            // 计算两个可能的差异
            double diff1 = std::abs(wrappedCurrent - wrappedPrevious);
            double diff2 = std::abs((wrappedCurrent - 2 * M_PI) - wrappedPrevious);

            // 选择较小的差异
            if (diff1 < diff2)
            {
                closestAngles[i] = wrappedCurrent;
            }
            else
            {
                closestAngles[i] = std::fmod(wrappedCurrent - 2 * M_PI + M_PI, 2 * M_PI) - M_PI;
            }
        }

        // 更新上一次的欧拉角状态
        previousAngles = closestAngles;

        return closestAngles;
    }
};
#endif //UTILFUNC_H
