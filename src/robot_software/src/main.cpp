//
// Created by lbt on 24-12-8.
//

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "robot_software/robot_estimator/RobotEstimatorNode.h"
#include "robot_software/robot_interface/MujocoInterface.h"

#include <pinocchio/fwd.hpp>

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"
///git test

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto Pin_ptr_ = std::make_shared<Galileo::PinocchioInterface>();

    const auto RobotInterfaceNode = std::make_shared<Galileo::MujocoInterface>(Pin_ptr_);
    const auto RobotEstimatorNode = std::make_shared<Galileo::RobotEstimatorNode>(Pin_ptr_);
    executor.add_node(RobotInterfaceNode);
    executor.add_node(RobotEstimatorNode);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
