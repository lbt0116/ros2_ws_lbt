//
// Created by lbt on 24-12-8.
//

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>

#include "robot_software/robot_FSM/RobotFSMNode.h"
#include "robot_software/robot_controller/RobotControllerNode.h"
#include "robot_software/robot_controller/nmpc_controller/GeometricNmpcNode.h"
// #include "robot_software/robot_controller/nmpc_controller/NmpcDdpCtrlNode.h"
#include "robot_software/robot_estimator/RobotEstimatorNode.h"
#include "robot_software/robot_interface/MujocoInterfaceNode.h"
#include "robot_software/robot_interface/PinocchioInterfaceNode.h"
#include "robot_software/robot_interface/RobotRecordNode.h"
#include "robot_software/robot_interface/UserInterfaceNode.h"
#include "robot_software/robot_planning/RobotPlanningNode.h"
int main(int argc, char* argv[])
{
    // 设置环境变量启用颜色输出
    setenv("RCUTILS_COLORIZED_OUTPUT", "1", 1);
    rclcpp::init(argc, argv);

    // 静态单线程执行器 省去动态管理节点的开销
    rclcpp::executors::StaticSingleThreadedExecutor main_executor;
    // 多线程执行器 用于用户交互
    rclcpp::executors::MultiThreadedExecutor user_executor;
    // 多线程执行器 用于用户交互 绑定4个线程
    // rclcpp::executors::MultiThreadedExecutor user_executor(rclcpp::ExecutorOptions(), 4);

    // 创建节点
    const auto RobotInterfaceNode = std::make_shared<Galileo::MujocoInterfaceNode>();
    const auto PinocchioInterfaceNode = std::make_shared<Galileo::PinocchioInterfaceNode>();
    const auto RobotEstimatorNode = std::make_shared<Galileo::RobotEstimatorNode>();
    const auto RobotFSMNode = std::make_shared<Galileo::RobotFSMNode>();
    const auto RobotPlanningNode = std::make_shared<Galileo::RobotPlanningNode>();
    const auto RobotControllerNode = std::make_shared<Galileo::RobotControllerNode>();
    const auto UserInterfaceNode = std::make_shared<Galileo::UserInterfaceNode>();
    const auto RobotRecordNode = std::make_shared<Galileo::RobotRecordNode>();
    const auto GeometricNmpcNode = std::make_shared<Galileo::GeometricNmpcNode>();
    // 添加节点到执行器
    main_executor.add_node(RobotInterfaceNode);
    main_executor.add_node(PinocchioInterfaceNode);
    main_executor.add_node(RobotEstimatorNode);
    main_executor.add_node(RobotFSMNode);
    main_executor.add_node(RobotPlanningNode);
    main_executor.add_node(RobotControllerNode);

    // 添加用户交互节点到多线程执行器
    user_executor.add_node(UserInterfaceNode);
    user_executor.add_node(RobotRecordNode);
    user_executor.add_node(GeometricNmpcNode);
    // 创建一个线程来运行用户交互执行器
    std::thread user_thread([&]() { user_executor.spin(); });

    // 在主线程中运行主执行器
    main_executor.spin();

    // 等待用户交互线程结束
    user_thread.join();

    // 关闭节点
    rclcpp::shutdown();
    return 0;
}
