//
// Created by lbt on 24-12-8.
//

#ifndef ROBOTESTIMATORNODE_H
#define ROBOTESTIMATORNODE_H
#include <Eigen/Dense>
#include <memory>  // 用于智能指针

#include "custom_msgs/msg/actuator_cmds.hpp"
#include "custom_msgs/msg/robot_state_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_software/robot_estimator/EskfOnSe3.h"
#include "robot_software/robot_estimator/LinearKalmanFilter.h"
#include "robot_software/robot_interface/PinocchioInterfaceNode.h"
#include "robot_software/robot_utils/MatrixTypes.h"
#include "robot_software/robot_utils/UtilFunc.h"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace Eigen;

namespace Galileo
{
class RobotEstimatorNode : public rclcpp::Node
{
public:
    RobotEstimatorNode();  //
    ~RobotEstimatorNode() override = default;

private:
    // 扩展卡尔曼滤波器
    std::shared_ptr<EskfOnSe3> eskf_;

    // 订阅触发信号
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr triggerSub_;

    // 触发信号回调函数
    void trigger_callback(const std_msgs::msg::Bool::ConstSharedPtr& msg);

    // 数据中心
    DataCenter& dataCenter;
};
}  // namespace Galileo

#endif  // ROBOTESTIMATORNODE_H
