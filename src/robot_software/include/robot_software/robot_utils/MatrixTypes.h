//
// Created by lbt on 24-12-9.
//

#ifndef MATRIXTYPES_H
#define MATRIXTYPES_H

#include "eigen3/Eigen/Dense"

namespace Galileo
{
    using mat43 = Eigen::Matrix<double, 4, 3>;
    using mat33 = Eigen::Matrix<double, 3, 3>;
    using mat66 = Eigen::Matrix<double, 6, 6>;
    using mat34 = Eigen::Matrix<double, 3, 4>;
    using vec12 = Eigen::Vector<double, 12>;
    using vec3 = Eigen::Vector<double, 3>;
    using vec4i = Eigen::Vector4i;
}


#endif //MATRIXTYPES_H
