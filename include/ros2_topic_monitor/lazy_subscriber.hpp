#ifndef ROS2_TOPIC_MONITOR__LAZY_SUBSCRIBER_HPP_
#define ROS2_TOPIC_MONITOR__LAZY_SUBSCRIBER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <string>
#include <memory>
#include <atomic>
#include <mutex>
#include "ros2_topic_monitor/metrics_manager.hpp"
#include "ros2_topic_monitor/message_type_loader.hpp"

namespace ros2_topic_monitor
{

class LazySubscriber
{
public:
  LazySubscriber(
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name,
    const std::string & topic_type);
  
  ~LazySubscriber() = default;

  // Enable detail mode (parse message content)
  void enableDetailMode();

  // Disable detail mode (only track metadata)
  void disableDetailMode();

  // Check if detail mode is enabled
  bool isDetailModeEnabled() const;

  // Get metrics manager
  std::shared_ptr<MetricsManager> getMetrics() const;

  // Get last message content (only available in detail mode)
  std::string getLastMessageContent() const;

  // Toggle array expansion in detail mode
  void toggleArrayExpansion();
  
  // Check if array expansion is enabled
  bool isArrayExpansionEnabled() const;

private:
  void messageCallback(std::shared_ptr<rclcpp::SerializedMessage> msg);
  
  rclcpp::Time extractTimestamp(const std::shared_ptr<rclcpp::SerializedMessage> & msg);
  
  // Get adaptive QoS profile based on existing publishers
  rclcpp::QoS getAdaptiveQoS(const std::string & topic_name);

  rclcpp::Node::SharedPtr node_;
  std::string topic_name_;
  std::string topic_type_;
  
  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  std::shared_ptr<MetricsManager> metrics_;
  std::shared_ptr<MessageTypeLoader> type_loader_;
  
  std::atomic<bool> detail_mode_enabled_{false};
  std::atomic<bool> expand_arrays_{false};
  
  mutable std::mutex content_mutex_;
  std::string last_message_content_;
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__LAZY_SUBSCRIBER_HPP_
