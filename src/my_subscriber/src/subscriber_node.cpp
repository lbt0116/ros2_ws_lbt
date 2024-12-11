#include <sensor_msgs/msg/detail/imu__builder.hpp>

#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/custom_message.hpp"
#include "sensor_msgs/msg/imu.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode() : Node("custom_subscriber")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data", qos, std::bind(&SubscriberNode::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    // RCLCPP_INFO(this->get_logger(), "sub imu node %.2f", msg->linear_acceleration.z);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
