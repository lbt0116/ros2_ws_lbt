//
// Created by lbt on 24-12-9.
//

#ifndef UTILFUNC_H
#define UTILFUNC_H

#include <Eigen/Dense>
#include <algorithm>
#include <concepts>
#include <execution>
#include <memory>
#include <ranges>
#include <span>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

/**
 * @brief 将Eigen矩阵高效转换为std::array
 * @tparam MatrixType 必须是Eigen::MatrixBase的派生类，且大小必须是编译时已知的固定大小
 * @param matrix 要转换的Eigen矩阵
 * @return 转换后的std::array
 * @throws std::invalid_argument 如果矩阵的大小在编译时不可知
 */
template <typename MatrixType>
requires std::is_base_of_v<Eigen::MatrixBase<MatrixType>, MatrixType>
    std::array<typename MatrixType::Scalar, MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime>
    eigenToStdArray(const MatrixType& matrix)
{
    using Scalar = typename MatrixType::Scalar;

    // 检查矩阵是否是固定大小
    static_assert(MatrixType::RowsAtCompileTime != Eigen::Dynamic && MatrixType::ColsAtCompileTime != Eigen::Dynamic,
                  "Matrix must have fixed size at compile time");

    // 创建 std::array 并直接拷贝数据
    constexpr std::size_t size = MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime;
    std::array<Scalar, size> result;
    Eigen::Map<Eigen::Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime, Eigen::ColMajor>>(
        result.data()) = matrix;

    return result;
}

/**
 * @brief 将std::array高效转换为Eigen矩阵
 * @tparam MatrixType 必须是Eigen::MatrixBase的派生类，且大小必须是编译时已知的固定大小
 * @param arr 要转换的std::array
 * @return 转换后的Eigen矩阵
 */
template <typename MatrixType>
requires std::is_base_of_v<Eigen::MatrixBase<MatrixType>, MatrixType> MatrixType stdArrayToEigen(
    const std::array<typename MatrixType::Scalar, MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime>& arr)
{
    using Scalar = typename MatrixType::Scalar;

    // 检查矩阵是否是固定大小
    static_assert(MatrixType::RowsAtCompileTime != Eigen::Dynamic && MatrixType::ColsAtCompileTime != Eigen::Dynamic,
                  "Matrix must have fixed size at compile time");

    // 使用 Eigen::Map 将 std::array 映射为 Eigen 矩阵
    constexpr std::size_t size = MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime;
    Eigen::Map<
        const Eigen::Matrix<Scalar, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime, Eigen::ColMajor>>
        mapped_matrix(arr.data());

    // 返回映射矩阵的副本
    return mapped_matrix;
}

/**
 * @brief 将Eigen矩阵/向量高效转换为std::vector
 * @tparam MatrixType Eigen矩阵/向量类型
 * @param matrix 输入的Eigen矩阵/向量
 * @return 转换后的std::vector
 *
 * @example
 * ```cpp
 * // 向量转换示例
 * Eigen::Vector3d vec3d(1.0, 2.0, 3.0);
 * std::vector<double> stdVec = eigenToStdVector(vec3d);
 *
 * // 矩阵转换示例
 * Eigen::Matrix2d mat2d;
 * mat2d << 1, 2, 3, 4;
 * std::vector<double> stdVecMat = eigenToStdVector(mat2d);
 * ```
 */
template <typename MatrixType>
requires std::is_base_of_v<Eigen::MatrixBase<MatrixType>, MatrixType> std::vector<typename MatrixType::Scalar>
eigenToStdVector(const MatrixType& matrix)
{
    using Scalar = typename MatrixType::Scalar;
    const size_t size = matrix.size();
    std::vector<Scalar> result(size);

    // 使用Eigen::Map直接访问内存，避免逐元素拷贝
    Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(result.data(), size) =
        Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>(matrix.data(), size);

    return result;
}

/**
 * @brief 将std::vector高效转换为Eigen矩阵/���量
 * @tparam MatrixType 目标Eigen矩阵/向量类型
 * @param vec 输入的std::vector
 * @return 转换后的Eigen矩阵/向量
 * @throws std::invalid_argument 如果vector的大小与目标矩阵不匹配
 *
 * @example
 * ```cpp
 * // 向量转换示例
 * std::vector<double> vec = {1.0, 2.0, 3.0};
 * Eigen::Vector3d eigenVec = stdVectorToEigen<Eigen::Vector3d>(vec);
 *
 * // 矩阵转换示例
 * std::vector<double> mat = {1.0, 2.0, 3.0, 4.0};
 * Eigen::Matrix2d eigenMat = stdVectorToEigen<Eigen::Matrix2d>(mat);
 *
 * // 动态大小矩阵示例
 * std::vector<double> dynVec = {1.0, 2.0, 3.0, 4.0, 5.0};
 * Eigen::VectorXd eigenDynVec = stdVectorToEigen<Eigen::VectorXd>(dynVec);
 * ```
 */
template <typename MatrixType>
requires std::is_base_of_v<Eigen::MatrixBase<MatrixType>, MatrixType> MatrixType
stdVectorToEigen(std::span<const typename MatrixType::Scalar> vec)
{
    using Scalar = typename MatrixType::Scalar;

    if constexpr (MatrixType::SizeAtCompileTime != Eigen::Dynamic)
    {
        if (vec.size() != MatrixType::SizeAtCompileTime)
        {
            throw std::invalid_argument("Vector size does not match matrix dimensions");
        }
    }

    MatrixType result;
    if constexpr (MatrixType::SizeAtCompileTime == Eigen::Dynamic)
    {
        result.resize(vec.size(), 1);
    }

    // 使用Eigen::Map实现零拷贝转换
    Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> mapped(vec.data(), vec.size());
    if constexpr (MatrixType::ColsAtCompileTime == 1)
    {
        result = mapped;
    }
    else
    {
        result = Eigen::Map<const MatrixType>(vec.data());
    }

    return result;
}

#include "eigen3/Eigen/Dense"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

// Sophus SE3 vee hat 向量顺序是先平移后旋转
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
        Eigen::Vector3d omega = xi.head<3>();  // 旋转部分
        Eigen::Vector3d v = xi.tail<3>();      // 平移部分

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

        Eigen::Matrix<double, 4, 4> g = (eye - 0.5 * hat2SE3(xi)).inverse() * (eye + 0.5 * hat2SE3(xi));
        // Eigen::Quaterniond q(g.topLeftCorner<3, 3>());
        // g.topLeftCorner<3, 3>() = q.toRotationMatrix(); //保证旋转矩阵正交，但是会损失一点精度

        return g;
    }

    static Eigen::Matrix<double, 6, 1> invCayley2se3(const Eigen::Matrix<double, 4, 4>& g)
    {
        const Eigen::Matrix<double, 4, 4> eye = Eigen::Matrix4d::Identity();

        Eigen::Matrix<double, 6, 1> xi = vee2se3(2 * (g - eye) * (g + eye).inverse());
        return xi;  // Rz!=pi
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
            double wrappedCurrent = std::fmod(currentAngles[i] + M_PI, 2 * M_PI) - M_PI;  // 将角度限制在[-π, π]
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
#endif  // UTILFUNC_H
