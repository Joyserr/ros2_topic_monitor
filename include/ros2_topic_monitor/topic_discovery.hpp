#ifndef ROS2_TOPIC_MONITOR__TOPIC_DISCOVERY_HPP_
#define ROS2_TOPIC_MONITOR__TOPIC_DISCOVERY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <shared_mutex>
#include <memory>

namespace ros2_topic_monitor
{

struct TopicInfo
{
  std::string name;
  std::string type;
  size_t publisher_count{0};
  size_t subscription_count{0};
  bool active{true};
  
  // QoS profile information (from first publisher/subscriber)
  std::string qos_reliability;
  std::string qos_durability;
  std::string qos_history;
  int qos_depth{0};
};

class TopicDiscovery
{
public:
  explicit TopicDiscovery(rclcpp::Node::SharedPtr node);
  ~TopicDiscovery() = default;

  // Refresh topic list from ROS graph
  void refreshTopics();

  // Get list of all topic names
  std::vector<std::string> getTopicList() const;

  // Get info for a specific topic
  TopicInfo getTopicInfo(const std::string & topic_name) const;

  // Check if topic exists
  bool hasTopic(const std::string & topic_name) const;

  // Get all topics info
  std::unordered_map<std::string, TopicInfo> getAllTopics() const;

private:
  rclcpp::Node::SharedPtr node_;
  mutable std::shared_mutex topics_mutex_;
  std::unordered_map<std::string, TopicInfo> topics_;
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__TOPIC_DISCOVERY_HPP_
