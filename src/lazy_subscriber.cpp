#include "ros2_topic_monitor/lazy_subscriber.hpp"

namespace ros2_topic_monitor
{

rclcpp::QoS LazySubscriber::getAdaptiveQoS(const std::string & topic_name)
{
  // Get information about existing publishers on this topic
  auto publishers_info = node_->get_publishers_info_by_topic(topic_name);
  
  // Default QoS settings (compatible with most scenarios)
  rclcpp::QoS qos_profile(10);
  
  if (publishers_info.empty()) {
    // No publishers yet, use a permissive default
    RCLCPP_WARN(
      node_->get_logger(),
      "No publishers found for topic '%s', using default QoS with BEST_EFFORT reliability",
      topic_name.c_str()
    );
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    return qos_profile;
  }
  
  // Adapt to the first publisher's QoS settings
  const auto & pub_qos = publishers_info[0].qos_profile();
  
  // Match reliability policy
  qos_profile.reliability(pub_qos.reliability());
  
  // Match durability policy
  qos_profile.durability(pub_qos.durability());
  
  // Match history policy
  if (pub_qos.history() == rclcpp::HistoryPolicy::KeepLast) {
    qos_profile.keep_last(pub_qos.depth());
  } else {
    qos_profile.keep_all();
  }
  
  // Match liveliness policy
  qos_profile.liveliness(pub_qos.liveliness());
  
  // For deadline and lifespan, use the publisher's settings
  qos_profile.deadline(pub_qos.deadline());
  qos_profile.lifespan(pub_qos.lifespan());
  
  RCLCPP_INFO(
    node_->get_logger(),
    "Adapted QoS from %zu publisher(s) on topic '%s'",
    publishers_info.size(),
    topic_name.c_str()
  );
  
  return qos_profile;
}

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
  
  // Create generic subscription with dynamic QoS
  auto callback = [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
    this->messageCallback(msg);
  };
  
  // Get dynamic QoS profile based on existing publishers
  rclcpp::QoS qos_profile = getAdaptiveQoS(topic_name_);
  
  RCLCPP_INFO(
    node_->get_logger(),
    "Creating subscription for topic '%s' with QoS: reliability=%s, durability=%s, history=%s",
    topic_name_.c_str(),
    qos_profile.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE ? "RELIABLE" : "BEST_EFFORT",
    qos_profile.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ? "TRANSIENT_LOCAL" : "VOLATILE",
    qos_profile.get_rmw_qos_profile().history == RMW_QOS_POLICY_HISTORY_KEEP_LAST ? "KEEP_LAST" : "KEEP_ALL"
  );
  
  subscription_ = node_->create_generic_subscription(
    topic_name_,
    topic_type_,
    qos_profile,
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

void LazySubscriber::toggleArrayExpansion()
{
  bool current = expand_arrays_.load(std::memory_order_relaxed);
  expand_arrays_.store(!current, std::memory_order_relaxed);
}

bool LazySubscriber::isArrayExpansionEnabled() const
{
  return expand_arrays_.load(std::memory_order_relaxed);
}

void LazySubscriber::messageCallback(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  // Extract timestamp
  rclcpp::Time msg_time = extractTimestamp(msg);
  
  // Update metrics (always) - this is lightweight
  metrics_->updateMetrics(msg, msg_time);
  

  if (detail_mode_enabled_.load(std::memory_order_relaxed)) {
    bool expand = expand_arrays_.load(std::memory_order_relaxed);
    std::string content = type_loader_->prettyPrint(msg, topic_type_, expand);
    
    std::lock_guard<std::mutex> content_lock(content_mutex_);
    last_message_content_ = std::move(content);
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
