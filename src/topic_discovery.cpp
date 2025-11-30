#include "ros2_topic_monitor/topic_discovery.hpp"
#include <algorithm>

namespace ros2_topic_monitor
{

TopicDiscovery::TopicDiscovery(rclcpp::Node::SharedPtr node)
: node_(node)
{
}

void TopicDiscovery::refreshTopics()
{
  auto topic_names_and_types = node_->get_topic_names_and_types();
  
  std::unique_lock<std::shared_mutex> lock(topics_mutex_);
  
  // Mark all topics as inactive first
  for (auto & [name, info] : topics_) {
    info.active = false;
  }
  
  // Update or add topics
  for (const auto & [topic_name, topic_types] : topic_names_and_types) {
    // Skip if no types
    if (topic_types.empty()) {
      continue;
    }
    
    // Use the first type
    std::string topic_type = topic_types[0];
    
    auto it = topics_.find(topic_name);
    if (it != topics_.end()) {
      // Update existing topic
      it->second.type = topic_type;
      it->second.active = true;
    } else {
      // Add new topic
      TopicInfo info;
      info.name = topic_name;
      info.type = topic_type;
      info.active = true;
      
      // Get publisher/subscriber count
      auto endpoints = node_->get_publishers_info_by_topic(topic_name);
      info.publisher_count = endpoints.size();
      
      auto sub_endpoints = node_->get_subscriptions_info_by_topic(topic_name);
      info.subscription_count = sub_endpoints.size();
      
      // Get QoS profile from first publisher (if available)
      if (!endpoints.empty()) {
        const auto & qos = endpoints[0].qos_profile();
        
        // Reliability
        switch (qos.reliability()) {
          case rclcpp::ReliabilityPolicy::BestEffort:
            info.qos_reliability = "BEST_EFFORT";
            break;
          case rclcpp::ReliabilityPolicy::Reliable:
            info.qos_reliability = "RELIABLE";
            break;
          default:
            info.qos_reliability = "UNKNOWN";
        }
        
        // Durability
        switch (qos.durability()) {
          case rclcpp::DurabilityPolicy::Volatile:
            info.qos_durability = "VOLATILE";
            break;
          case rclcpp::DurabilityPolicy::TransientLocal:
            info.qos_durability = "TRANSIENT_LOCAL";
            break;
          default:
            info.qos_durability = "UNKNOWN";
        }
        
        // History
        switch (qos.history()) {
          case rclcpp::HistoryPolicy::KeepLast:
            info.qos_history = "KEEP_LAST";
            info.qos_depth = qos.depth();
            break;
          case rclcpp::HistoryPolicy::KeepAll:
            info.qos_history = "KEEP_ALL";
            break;
          default:
            info.qos_history = "UNKNOWN";
        }
      } else if (!sub_endpoints.empty()) {
        // If no publishers, try to get QoS from first subscriber
        const auto & qos = sub_endpoints[0].qos_profile();
        
        // Reliability
        switch (qos.reliability()) {
          case rclcpp::ReliabilityPolicy::BestEffort:
            info.qos_reliability = "BEST_EFFORT";
            break;
          case rclcpp::ReliabilityPolicy::Reliable:
            info.qos_reliability = "RELIABLE";
            break;
          default:
            info.qos_reliability = "UNKNOWN";
        }
        
        // Durability
        switch (qos.durability()) {
          case rclcpp::DurabilityPolicy::Volatile:
            info.qos_durability = "VOLATILE";
            break;
          case rclcpp::DurabilityPolicy::TransientLocal:
            info.qos_durability = "TRANSIENT_LOCAL";
            break;
          default:
            info.qos_durability = "UNKNOWN";
        }
        
        // History
        switch (qos.history()) {
          case rclcpp::HistoryPolicy::KeepLast:
            info.qos_history = "KEEP_LAST";
            info.qos_depth = qos.depth();
            break;
          case rclcpp::HistoryPolicy::KeepAll:
            info.qos_history = "KEEP_ALL";
            break;
          default:
            info.qos_history = "UNKNOWN";
        }
      }
      
      topics_[topic_name] = info;
    }
  }
  
  // Remove inactive topics
  for (auto it = topics_.begin(); it != topics_.end(); ) {
    if (!it->second.active) {
      it = topics_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<std::string> TopicDiscovery::getTopicList() const
{
  std::shared_lock<std::shared_mutex> lock(topics_mutex_);
  
  std::vector<std::string> result;
  result.reserve(topics_.size());
  
  for (const auto & [name, info] : topics_) {
    if (info.active) {
      result.push_back(name);
    }
  }
  
  std::sort(result.begin(), result.end());
  return result;
}

TopicInfo TopicDiscovery::getTopicInfo(const std::string & topic_name) const
{
  std::shared_lock<std::shared_mutex> lock(topics_mutex_);
  
  auto it = topics_.find(topic_name);
  if (it != topics_.end()) {
    return it->second;
  }
  
  return TopicInfo{};
}

bool TopicDiscovery::hasTopic(const std::string & topic_name) const
{
  std::shared_lock<std::shared_mutex> lock(topics_mutex_);
  return topics_.find(topic_name) != topics_.end();
}

std::unordered_map<std::string, TopicInfo> TopicDiscovery::getAllTopics() const
{
  std::shared_lock<std::shared_mutex> lock(topics_mutex_);
  return topics_;
}

}  // namespace ros2_topic_monitor
