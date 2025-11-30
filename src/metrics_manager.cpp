#include "ros2_topic_monitor/metrics_manager.hpp"
#include <algorithm>

namespace ros2_topic_monitor
{

MetricsManager::MetricsManager()
{
  last_receive_time_ = std::chrono::system_clock::now();
  last_fps_update_ = std::chrono::system_clock::now();
}

void MetricsManager::updateMetrics(
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
  const rclcpp::Time & msg_timestamp)
{
  auto now = std::chrono::system_clock::now();
  
  {
    std::lock_guard<std::mutex> lock(time_mutex_);
    last_receive_time_ = now;
  }
  
  // Update message count
  message_count_.fetch_add(1, std::memory_order_relaxed);
  msgs_in_window_++;
  
  // Calculate delay
  if (msg_timestamp.nanoseconds() > 0) {
    auto msg_time = std::chrono::nanoseconds(msg_timestamp.nanoseconds());
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      now.time_since_epoch());
    
    auto delay_ns = now_ns - msg_time;
    double delay_ms = std::chrono::duration<double, std::milli>(delay_ns).count();
    
    // Apply EMA filter
    double current_delay = delay_.load(std::memory_order_relaxed);
    double new_delay = applyEMA(current_delay, delay_ms);
    delay_.store(new_delay, std::memory_order_relaxed);
  }
  
  // Update bandwidth
  if (serialized_msg) {
    size_t msg_size = serialized_msg->size();
    bytes_in_window_ += msg_size;
  }
  
  // Calculate FPS and bandwidth every second
  auto time_since_fps_update = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_fps_update_);
  
  if (time_since_fps_update.count() >= 1000) {
    // Calculate FPS
    double duration_sec = time_since_fps_update.count() / 1000.0;
    double new_fps = msgs_in_window_ / duration_sec;
    
    double current_fps = fps_.load(std::memory_order_relaxed);
    fps_.store(applyEMA(current_fps, new_fps), std::memory_order_relaxed);
    
    // Calculate bandwidth
    double new_bw = bytes_in_window_ / duration_sec;
    double current_bw = bandwidth_.load(std::memory_order_relaxed);
    bandwidth_.store(applyEMA(current_bw, new_bw), std::memory_order_relaxed);
    
    // Reset window
    msgs_in_window_ = 0;
    bytes_in_window_ = 0;
    last_fps_update_ = now;
  }
}

double MetricsManager::getFPS() const
{
  return fps_.load(std::memory_order_relaxed);
}

double MetricsManager::getDelay() const
{
  return delay_.load(std::memory_order_relaxed);
}

double MetricsManager::getBandwidth() const
{
  return bandwidth_.load(std::memory_order_relaxed);
}

std::chrono::system_clock::time_point MetricsManager::getLastReceiveTime() const
{
  std::lock_guard<std::mutex> lock(time_mutex_);
  return last_receive_time_;
}

uint64_t MetricsManager::getMessageCount() const
{
  return message_count_.load(std::memory_order_relaxed);
}

void MetricsManager::reset()
{
  fps_.store(0.0, std::memory_order_relaxed);
  delay_.store(0.0, std::memory_order_relaxed);
  bandwidth_.store(0.0, std::memory_order_relaxed);
  message_count_.store(0, std::memory_order_relaxed);
  
  bytes_in_window_ = 0;
  msgs_in_window_ = 0;
  
  std::lock_guard<std::mutex> lock(time_mutex_);
  last_receive_time_ = std::chrono::system_clock::now();
  last_fps_update_ = std::chrono::system_clock::now();
}

double MetricsManager::applyEMA(double current, double new_value, double alpha) const
{
  if (current == 0.0) {
    return new_value;
  }
  return alpha * new_value + (1.0 - alpha) * current;
}

}  // namespace ros2_topic_monitor
