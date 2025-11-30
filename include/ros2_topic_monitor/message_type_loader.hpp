#ifndef ROS2_TOPIC_MONITOR__MESSAGE_TYPE_LOADER_HPP_
#define ROS2_TOPIC_MONITOR__MESSAGE_TYPE_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <string>
#include <memory>

namespace ros2_topic_monitor
{

class MessageTypeLoader
{
public:
  MessageTypeLoader() = default;
  ~MessageTypeLoader() = default;

  // Pretty print a serialized message
  std::string prettyPrint(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
    const std::string & type_name);

private:
  // Truncate output to max lines and chars per line
  std::string truncateOutput(
    const std::string & input,
    size_t max_lines = 200,
    size_t max_chars_per_line = 120) const;
  
  // Convert binary data to hex string
  std::string toHexString(const uint8_t * data, size_t size, size_t max_bytes = 1024) const;
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__MESSAGE_TYPE_LOADER_HPP_
