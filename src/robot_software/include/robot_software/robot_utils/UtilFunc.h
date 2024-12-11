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
    matrix_out.derived() = Eigen::Map<const Eigen::Matrix<double, Derived::RowsAtCompileTime,
                                                          Derived::ColsAtCompileTime>>(
        msg.data.data(), msg.layout.dim[0].size, msg.layout.dim[1].size);
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
#endif //UTILFUNC_H
