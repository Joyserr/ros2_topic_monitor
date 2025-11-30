#ifndef ROS2_TOPIC_MONITOR__METRICS_MANAGER_HPP_
#define ROS2_TOPIC_MONITOR__METRICS_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <string>
#include <mutex>

namespace ros2_topic_monitor
{

class MetricsManager
{
public:
  MetricsManager();
  ~MetricsManager() = default;

  // Update metrics with new message
  void updateMetrics(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
    const rclcpp::Time & msg_timestamp);

  // Get frames per second
  double getFPS() const;

  // Get average delay in milliseconds
  double getDelay() const;

  // Get bandwidth in bytes/sec
  double getBandwidth() const;

  // Get last receive time
  std::chrono::system_clock::time_point getLastReceiveTime() const;

  // Get message count
  uint64_t getMessageCount() const;

  // Reset all metrics
  void reset();

private:
  // EMA (Exponential Moving Average) filter
  double applyEMA(double current, double new_value, double alpha = 0.2) const;

  std::atomic<double> fps_{0.0};
  std::atomic<double> delay_{0.0};
  std::atomic<double> bandwidth_{0.0};
  std::atomic<uint64_t> message_count_{0};
  
  mutable std::mutex time_mutex_;
  std::chrono::system_clock::time_point last_receive_time_;
  std::chrono::system_clock::time_point last_fps_update_;
  
  size_t bytes_in_window_{0};
  size_t msgs_in_window_{0};
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__METRICS_MANAGER_HPP_
