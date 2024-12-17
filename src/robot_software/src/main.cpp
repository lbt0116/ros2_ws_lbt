//
// Created by lbt on 24-12-8.
//

#include <pinocchio/fwd.hpp>

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_controller/RobotControllerNode.h"  // todo cxx20 datacenter test
#include "robot_software/robot_estimator/RobotEstimatorNode.h"
#include "robot_software/robot_interface/MujocoInterface.h"
#include "robot_software/robot_utils/DataCenter.hpp"  // todo cxx20 datacenter test
#include "sensor_msgs/msg/imu.hpp"
/// git test

int main(int argc, char* argv[])
{
    // 设置环境变量启用颜色输出
    setenv("RCUTILS_COLORIZED_OUTPUT", "1", 1);
    rclcpp::init(argc, argv);

    // 静态单线程执行器 省去动态管理节点的开销
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    const auto RobotInterfaceNode = std::make_shared<Galileo::MujocoInterface>();
    const auto PinocchioInterfaceNode = std::make_shared<Galileo::PinocchioInterface>();
    const auto RobotEstimatorNode = std::make_shared<Galileo::RobotEstimatorNode>();
    const auto RobotControllerNode = std::make_shared<Galileo::RobotControllerNode>();
    executor.add_node(RobotInterfaceNode);
    executor.add_node(PinocchioInterfaceNode);
    executor.add_node(RobotEstimatorNode);
    executor.add_node(RobotControllerNode);

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
