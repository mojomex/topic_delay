#include "topic_delay/topic_delay.hpp"

namespace nebula
{
namespace ros
{
TopicDelay::TopicDelay(const rclcpp::NodeOptions & options)
: rclcpp::Node("topic_delay", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  auto packets_topic = declare_parameter<std::string>("packets_topic", "");
  auto cloud_topic = declare_parameter<std::string>("cloud_topic", "");
  
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  if (cloud_topic.length() > 0) {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, qos,
      std::bind(&TopicDelay::ReceiveCloudMsgCallback, this, std::placeholders::_1));
  }

  if (packets_topic.length() > 0) {
    packets_sub_ = create_subscription<pandar_msgs::msg::PandarScan>(
      packets_topic, qos,
      std::bind(&TopicDelay::ReceiveScanMsgCallback, this, std::placeholders::_1));
  }
}

void TopicDelay::ReceiveCloudMsgCallback(const sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg)
{
  auto now = std::chrono::high_resolution_clock::now();
  uint64_t now_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  uint64_t msg_ns = static_cast<uint64_t>(cloud_msg->header.stamp.sec) * 1'000'000'000ULL +
                     static_cast<uint64_t>(cloud_msg->header.stamp.nanosec);

  RCLCPP_INFO_STREAM(
    get_logger(), "#cloud:" << cloud_sub_->get_topic_name() << ": " << std::fixed << std::setw(10) << std::setprecision(6)
                          << (now_ns - msg_ns) / 1'000'000. << "ms");
}

void TopicDelay::ReceiveScanMsgCallback(const pandar_msgs::msg::PandarScan::UniquePtr packets_msg)
{
  auto now = std::chrono::high_resolution_clock::now();
  uint64_t now_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  uint64_t msg_ns = static_cast<uint64_t>(packets_msg->header.stamp.sec) * 1'000'000'000ULL +
                     static_cast<uint64_t>(packets_msg->header.stamp.nanosec);

  RCLCPP_INFO_STREAM(
    get_logger(), "#packets:" << packets_sub_->get_topic_name() << ": " << std::fixed << std::setw(10) << std::setprecision(6)
                          << (now_ns - msg_ns) / 1'000'000. << "ms");
}

RCLCPP_COMPONENTS_REGISTER_NODE(TopicDelay)
}  // namespace ros
}  // namespace nebula
