#include "ros2_topic_monitor/message_type_loader.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace ros2_topic_monitor
{

std::string MessageTypeLoader::prettyPrint(
  const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
  const std::string & type_name)
{
  if (!serialized_msg) {
    return "[No message data]";
  }

  std::ostringstream oss;
  oss << "Message Type: " << type_name << "\n";
  oss << "Message Size: " << serialized_msg->size() << " bytes\n";
  oss << "---\n";
  
  // For now, display as hex dump since we don't have dynamic deserialization
  // In a full implementation, you would use rosidl_typesupport to deserialize
  const auto & rcl_msg = serialized_msg->get_rcl_serialized_message();
  std::string hex_dump = toHexString(
    rcl_msg.buffer,
    rcl_msg.buffer_length,
    1024  // Limit to first 1KB
  );
  
  oss << "Raw Data (hex):\n" << hex_dump;
  
  return truncateOutput(oss.str());
}

std::string MessageTypeLoader::truncateOutput(
  const std::string & input,
  size_t max_lines,
  size_t max_chars_per_line) const
{
  std::istringstream iss(input);
  std::ostringstream oss;
  std::string line;
  size_t line_count = 0;
  
  while (std::getline(iss, line) && line_count < max_lines) {
    if (line.length() > max_chars_per_line) {
      oss << line.substr(0, max_chars_per_line) << "...\n";
    } else {
      oss << line << "\n";
    }
    line_count++;
  }
  
  if (line_count >= max_lines) {
    oss << "... (truncated, " << max_lines << " lines shown)\n";
  }
  
  return oss.str();
}

std::string MessageTypeLoader::toHexString(
  const uint8_t * data,
  size_t size,
  size_t max_bytes) const
{
  std::ostringstream oss;
  size_t bytes_to_show = std::min(size, max_bytes);
  
  for (size_t i = 0; i < bytes_to_show; i++) {
    if (i > 0 && i % 16 == 0) {
      oss << "\n";
    } else if (i > 0 && i % 8 == 0) {
      oss << "  ";
    } else if (i > 0) {
      oss << " ";
    }
    
    oss << std::hex << std::setw(2) << std::setfill('0') 
        << static_cast<int>(data[i]);
  }
  
  if (size > max_bytes) {
    oss << "\n... (" << (size - max_bytes) << " more bytes)";
  }
  
  return oss.str();
}

}  // namespace ros2_topic_monitor
