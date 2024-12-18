#include "robot_software/robot_interface/UserInterfaceNode.h"

namespace Galileo
{
UserInterfaceNode::UserInterfaceNode()
    : Node("user_interface_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
}
}  // namespace Galileo