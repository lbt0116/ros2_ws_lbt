#include "robot_software/robot_interface/RobotRecordNode.h"

namespace Galileo
{
RobotRecordNode::RobotRecordNode()
    : Node("robot_record_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
    writer_ = std::make_shared<rosbag2_cpp::Writer>();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "record/record_bag_" + std::to_string(std::time(nullptr));
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";
    writer_->open(storage_options, converter_options);

    // 创建自定义 QoS 配置
    // qos.reliable();
    // qos.transient_local();

    // 获取所有活动主题并订阅
    auto topics_and_types = this->get_topic_names_and_types();
    for (const auto &pair : topics_and_types)
    {
        const auto &topic_name = pair.first;
        const auto &type_name = pair.second.front();  // 获取第一个类型
        writer_->create_topic({topic_name, type_name, "cdr", ""});
        RCLCPP_INFO(this->get_logger(), "Recording topic: %s, type: %s", topic_name.c_str(), type_name.c_str());

        // 为每个主题创建动态订阅
        subscribe_to_topic(topic_name, type_name);
    }

    RCLCPP_INFO(this->get_logger(), "RobotRecordNode initialized");
}

RobotRecordNode::~RobotRecordNode()
{
    writer_.reset();
    RCLCPP_INFO(this->get_logger(), "RobotRecordNode destroyed");
}

// 动态订阅每个主题
void RobotRecordNode::subscribe_to_topic(const std::string &topic_name, const std::string &topic_type)
{
    // 创建一个通用的QoS配置
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    // 创建一个动态订阅
    auto subscription = this->create_generic_subscription(
        topic_name,
        topic_type,
        qos,
        [this, topic_name, topic_type](std::shared_ptr<rclcpp::SerializedMessage> serialized_msg)
        {
            // 将接收到的消息写入Bag文件
            writer_->write(serialized_msg, topic_name, topic_type, this->get_clock()->now());
        });

    subscriptions_.push_back(subscription);  // 保存订阅对象，防止被销毁
}

std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;  // 保存所有订阅对象
}  // namespace Galileo