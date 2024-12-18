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
#include "robot_software/robot_FSM/RobotFSMNode.h"
#include "robot_software/robot_controller/RobotControllerNode.h"
#include "robot_software/robot_estimator/RobotEstimatorNode.h"
#include "robot_software/robot_interface/MujocoInterface.h"
#include "robot_software/robot_planning/RobotPlanningNode.h"
int main(int argc, char* argv[])
{
    // 设置环境变量启用颜色输出
    setenv("RCUTILS_COLORIZED_OUTPUT", "1", 1);
    rclcpp::init(argc, argv);

    // 静态单线程执行器 省去动态管理节点的开销
    rclcpp::executors::StaticSingleThreadedExecutor executor;

    // 创建节点
    const auto RobotInterfaceNode = std::make_shared<Galileo::MujocoInterface>();
    const auto PinocchioInterfaceNode = std::make_shared<Galileo::PinocchioInterface>();
    const auto RobotEstimatorNode = std::make_shared<Galileo::RobotEstimatorNode>();
    const auto RobotFSMNode = std::make_shared<Galileo::RobotFSMNode>();
    const auto RobotPlanningNode = std::make_shared<Galileo::RobotPlanningNode>();
    const auto RobotControllerNode = std::make_shared<Galileo::RobotControllerNode>();

    // 添加节点到执行器
    executor.add_node(RobotInterfaceNode);
    executor.add_node(PinocchioInterfaceNode);
    executor.add_node(RobotEstimatorNode);
    executor.add_node(RobotFSMNode);
    executor.add_node(RobotPlanningNode);
    executor.add_node(RobotControllerNode);
    // 启动执行器
    executor.spin();

    // 关闭节点
    rclcpp::shutdown();
    return 0;
}
