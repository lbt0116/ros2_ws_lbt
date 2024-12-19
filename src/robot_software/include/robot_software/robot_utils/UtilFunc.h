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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

template <typename Derived>
void eigenToFloat64MultiArray(const Eigen::MatrixBase<Derived>& matrix,
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
    Eigen::Map<Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>>(
        msg.data.data(), matrix.rows(), matrix.cols()) = matrix;
}

// 反向转换函数
template <typename Derived>
void float64MultiArrayToEigen(const std_msgs::msg::Float64MultiArray& msg,
                              Eigen::MatrixBase<Derived>& matrix_out)
{
    matrix_out.derived() = Eigen::Map<
        const Eigen::Matrix<double, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>>(
        msg.data.data(), msg.layout.dim[0].size, msg.layout.dim[1].size);
}

/**
 * @brief 将Eigen矩阵转换为std::vector
 * @tparam MatrixType 必须是Eigen::MatrixBase的派生类
 * @param matrix 要转换的Eigen矩阵
 * @return 转换后的std::vector
 * @throws std::invalid_argument 如果输入矩阵为空
 */
template <typename MatrixType>
requires std::is_base_of_v<Eigen::MatrixBase<MatrixType>, MatrixType>
auto eigenToStdVector(const MatrixType& matrix) -> std::vector<typename MatrixType::Scalar>
{
    using Scalar = typename MatrixType::Scalar;

    // 检查矩阵是否为空
    if (matrix.size() == 0)
    {
        throw std::invalid_argument("The input Eigen matrix is empty (size 0).");
    }

    // 分配 std::vector 并将矩阵转换为一维存储
    std::vector<Scalar> result(matrix.size());

    try
    {
        // 使用 C++20 的 ranges::transform 和 reshaped
        std::ranges::transform(
            matrix.reshaped(), result.begin(), [](const Scalar& value) { return value; });
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(std::string("Error during Eigen to std::vector conversion: ")
                                 + e.what());
    }

    return result;
}

/**
 * @brief 将std::vector转换为Eigen矩阵
 * @tparam MatrixType 必须是Eigen::MatrixBase的派生类
 * @param vec 要转换的std::vector
 * @param rows 目标矩阵的行数
 * @param cols 目标矩阵的列数
 * @return 转换后的Eigen矩阵
 * @throws std::invalid_argument 如果输入的std::vector为空或大小不匹配
 */
template <typename MatrixType>
requires std::is_base_of_v<Eigen::MatrixBase<MatrixType>, MatrixType> MatrixType
stdVectorToEigen(const std::vector<typename MatrixType::Scalar>& vec, int rows, int cols)
{
    using Scalar = typename MatrixType::Scalar;

    // 检查矩阵的行列大小是否合理
    if (rows <= 0 || cols <= 0)
    {
        throw std::invalid_argument(
            "Invalid matrix dimensions: rows and cols must be greater than 0.");
    }

    // 检查 std::vector 的大小是否与目标矩阵匹配
    if (vec.size() != static_cast<size_t>(rows * cols))
    {
        throw std::invalid_argument(
            "The size of the std::vector does not match the specified matrix dimensions.");
    }

    // 创建矩阵并填充数据
    MatrixType matrix(rows, cols);
    try
    {
        for (int i = 0; i < rows; ++i)
        {
            for (int j = 0; j < cols; ++j)
            {
                // 按照 Column-Major 存储，将 vec 映射到 Eigen 矩阵
                matrix(i, j) = vec[i + j * rows];
            }
        }
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(std::string("Error during std::vector to Eigen conversion: ")
                                 + e.what());
    }

    return matrix;
}

/**
 * @brief 将Eigen矩阵高效转换为std::array
 * @tparam MatrixType 必须是Eigen::MatrixBase的派生类，且大小必须是编译时已知的固定大小
 * @param matrix 要转换的Eigen矩阵
 * @return 转换后的std::array
 * @throws std::invalid_argument 如果矩阵的大小在编译时不可知
 */
template <typename MatrixType>
requires std::is_base_of_v<Eigen::MatrixBase<MatrixType>, MatrixType>
    std::array<typename MatrixType::Scalar,
               MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime>
    eigenToStdArray(const MatrixType& matrix)
{
    using Scalar = typename MatrixType::Scalar;

    // 检查矩阵是否是固定大小
    static_assert(MatrixType::RowsAtCompileTime != Eigen::Dynamic
                      && MatrixType::ColsAtCompileTime != Eigen::Dynamic,
                  "Matrix must have fixed size at compile time");

    // 创建 std::array 并直接拷贝数据
    constexpr std::size_t size = MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime;
    std::array<Scalar, size> result;
    Eigen::Map<Eigen::Matrix<Scalar,
                             MatrixType::RowsAtCompileTime,
                             MatrixType::ColsAtCompileTime,
                             Eigen::ColMajor>>(result.data()) = matrix;

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
    const std::array<typename MatrixType::Scalar,
                     MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime>& arr)
{
    using Scalar = typename MatrixType::Scalar;

    // 检查矩阵是否是固定大小
    static_assert(MatrixType::RowsAtCompileTime != Eigen::Dynamic
                      && MatrixType::ColsAtCompileTime != Eigen::Dynamic,
                  "Matrix must have fixed size at compile time");

    // 使用 Eigen::Map 将 std::array 映射为 Eigen 矩阵
    constexpr std::size_t size = MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime;
    Eigen::Map<const Eigen::Matrix<Scalar,
                                   MatrixType::RowsAtCompileTime,
                                   MatrixType::ColsAtCompileTime,
                                   Eigen::ColMajor>>
        mapped_matrix(arr.data());

    // 返回映射矩阵的副本
    return mapped_matrix;
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

        Eigen::Matrix<double, 4, 4> g =
            (eye - 0.5 * hat2SE3(xi)).inverse() * (eye + 0.5 * hat2SE3(xi));
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
            double wrappedCurrent =
                std::fmod(currentAngles[i] + M_PI, 2 * M_PI) - M_PI;  // 将角度限制在[-π, π]
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
