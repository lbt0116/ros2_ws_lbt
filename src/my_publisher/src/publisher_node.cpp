#include "custom_msgs/msg/actuator_cmds.hpp"
#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

class MujocoNode : public rclcpp::Node
{
public:
    MujocoNode()
        : Node("custom_publisher")
    {
        auto qos   = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
        publisher_ = this->create_publisher<custom_msgs::msg::ActuatorCmds>("actuators_cmds", qos);
        timer_     = this->create_wall_timer(1ms, std::bind(&MujocoNode::publish_message, this));
    }

private:
    void publish_message()
    {
        auto msg           = custom_msgs::msg::ActuatorCmds();
        msg.actuators_name = {
            "Abd1Joint",
            "Hip1Joint",
            "Knee1Joint",
            "Abd2Joint",
            "Hip2Joint",
            "Knee2Joint",
            "Abd3Joint",
            "Hip3Joint",
            "Knee3Joint",
            "Abd4Joint",
            "Hip4Joint",
            "Knee4Joint"};
        // 示例数据
        msg.torque.resize(msg.actuators_name.size());
        msg.torque_limit.resize(msg.actuators_name.size());
        msg.pos.resize(msg.actuators_name.size());
        msg.vel.resize(msg.actuators_name.size());
        msg.kp.resize(msg.actuators_name.size());
        msg.kd.resize(msg.actuators_name.size());
        msg.pos_limit.resize(msg.actuators_name.size());
        for (size_t k = 0; k < msg.actuators_name.size(); k++)
        {
            msg.torque[k]       = 1;
            msg.torque_limit[k] = 100;
            msg.pos_limit[k]    = 100;
            msg.pos[k]          = 0;
            msg.vel[k]          = 0;
            msg.kp[k]           = 0;
            msg.kd[k]           = 0;
        }

        publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing torque: '%f'", msg.torque[0]);
    }

    rclcpp::Publisher<custom_msgs::msg::ActuatorCmds>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr                                 timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MujocoNode>());
    rclcpp::shutdown();
    return 0;
}
