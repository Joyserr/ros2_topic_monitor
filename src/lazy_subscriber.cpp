#include "ros2_topic_monitor/lazy_subscriber.hpp"

namespace ros2_topic_monitor
{

LazySubscriber::LazySubscriber(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  const std::string & topic_type)
: node_(node),
  topic_name_(topic_name),
  topic_type_(topic_type)
{
  metrics_ = std::make_shared<MetricsManager>();
  type_loader_ = std::make_shared<MessageTypeLoader>();
  
  // Create generic subscription
  auto callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
    this->messageCallback(msg);
  };
  
  subscription_ = node_->create_generic_subscription(
    topic_name_,
    topic_type_,
    rclcpp::QoS(10),
    callback
  );
}

void LazySubscriber::enableDetailMode()
{
  detail_mode_enabled_.store(true, std::memory_order_relaxed);
}

void LazySubscriber::disableDetailMode()
{
  detail_mode_enabled_.store(false, std::memory_order_relaxed);
  
  std::lock_guard<std::mutex> lock(content_mutex_);
  last_message_content_.clear();
}

bool LazySubscriber::isDetailModeEnabled() const
{
  return detail_mode_enabled_.load(std::memory_order_relaxed);
}

std::shared_ptr<MetricsManager> LazySubscriber::getMetrics() const
{
  return metrics_;
}

std::string LazySubscriber::getLastMessageContent() const
{
  std::lock_guard<std::mutex> lock(content_mutex_);
  return last_message_content_;
}

void LazySubscriber::messageCallback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  // Extract timestamp
  rclcpp::Time msg_time = extractTimestamp(msg);
  
  // Update metrics (always)
  metrics_->updateMetrics(msg, msg_time);
  
  // Parse message content only in detail mode
  if (detail_mode_enabled_.load(std::memory_order_relaxed)) {
    std::string content = type_loader_->prettyPrint(msg, topic_type_);
    
    std::lock_guard<std::mutex> lock(content_mutex_);
    last_message_content_ = content;
  }
}

rclcpp::Time LazySubscriber::extractTimestamp(
  const std::shared_ptr<rclcpp::SerializedMessage> & msg)
{
  // Try to extract timestamp from message header
  // For messages without header, return zero time
  // This is a simplified implementation
  // In a full version, you would use rosidl_typesupport to properly deserialize
  
  if (!msg || msg->size() < 8) {
    return rclcpp::Time(0);
  }
  
  // Many ROS2 messages have timestamp in first 8 bytes after CDR header
  // This is a heuristic and may not work for all message types
  const auto & rcl_msg = msg->get_rcl_serialized_message();
  
  // Skip CDR header (4 bytes) and try to read timestamp
  if (rcl_msg.buffer_length >= 16) {
    // This is a simplified extraction - real implementation would need
    // proper CDR deserialization
    return rclcpp::Time(0);  // Return zero for now
  }
  
  return rclcpp::Time(0);
}

}  // namespace ros2_topic_monitor
