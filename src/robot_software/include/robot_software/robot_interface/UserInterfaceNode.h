#pragma once

#include <rclcpp/rclcpp.hpp>

namespace Galileo
{
class UserInterfaceNode : public rclcpp::Node
{
public:
    UserInterfaceNode();
    ~UserInterfaceNode() override = default;
};
}  // namespace Galileo