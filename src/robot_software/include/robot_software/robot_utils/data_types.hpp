#pragma once
#include <array>
#include <Eigen/Dense>

namespace Galileo {

struct Position {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    // 转换为 Eigen::Vector3d
    Eigen::Vector3d toEigen() const {
        return Eigen::Vector3d(x, y, z);
    }

    // 从 Eigen::Vector3d 构造
    static Position fromEigen(const Eigen::Vector3d& vec) {
        return Position{vec.x(), vec.y(), vec.z()};
    }
};

struct Velocity {
    double vx{0.0};
    double vy{0.0};
    double vz{0.0};

    // 转换为 Eigen::Vector3d
    Eigen::Vector3d toEigen() const {
        return Eigen::Vector3d(vx, vy, vz);
    }

    // 从 Eigen::Vector3d 构造
    static Velocity fromEigen(const Eigen::Vector3d& vec) {
        return Velocity{vec.x(), vec.y(), vec.z()};
    }
};

struct EstimatorData {
    Position position;
    Velocity velocity;
    double timestamp{0.0};
};

// 为了方便使用 Eigen 类型，添加常用的类型别名
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

} // namespace Galileo 