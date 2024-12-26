#pragma once

#include <rclcpp/rclcpp.hpp>

#include "robot_software/robot_utils/DataCenter.hpp"

namespace Galileo
{
class SwingLegController
{
public:
    SwingLegController();
    ~SwingLegController();

    void run();
    void declare_and_get_parameters(rclcpp::Node* node);

    void set_parameters(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

private:
    mat34 kp;
    mat34 kd;
    DataCenter& dataCenter_;
};

}  // namespace Galileo