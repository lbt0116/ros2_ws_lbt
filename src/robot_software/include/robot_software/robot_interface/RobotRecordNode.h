#pragma once

#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include "rclcpp/rclcpp.hpp"
namespace Galileo
{
class RobotRecordNode : public rclcpp::Node
{
public:
    RobotRecordNode();
    ~RobotRecordNode();

private:
    void subscribe_to_topic(const std::string& topic_name, const std::string& topic_type);
    std::shared_ptr<rosbag2_cpp::Writer> writer_;
    std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
};
}  // namespace Galileo
