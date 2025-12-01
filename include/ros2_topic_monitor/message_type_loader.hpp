#ifndef ROS2_TOPIC_MONITOR__MESSAGE_TYPE_LOADER_HPP_
#define ROS2_TOPIC_MONITOR__MESSAGE_TYPE_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <string>
#include <memory>
#include <vector>
#include <sstream>
#include <map>

namespace ros2_topic_monitor
{

class MessageTypeLoader
{
public:
  MessageTypeLoader() = default;
  ~MessageTypeLoader() = default;

  // Pretty print a serialized message with structure parsing
  std::string prettyPrint(
    const std::shared_ptr<rclcpp::SerializedMessage> & serialized_msg,
    const std::string & type_name,
    bool expand_arrays = false);

private:
  // Parse message using introspection
  std::string parseMessage(
    const void * message_data,
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    int indent_level,
    bool expand_arrays);
  
  // Parse a single field
  std::string parseField(
    const void * field_data,
    const rosidl_typesupport_introspection_cpp::MessageMember * member,
    int indent_level,
    bool expand_arrays);
  
  // Parse array field
  std::string parseArrayField(
    const void * field_data,
    const rosidl_typesupport_introspection_cpp::MessageMember * member,
    int indent_level,
    bool expand_arrays);
  
  // Format field name with highlighting for important fields
  std::string formatFieldName(
    const std::string & field_name,
    bool is_key_field = false) const;
  
  // Check if field is a key field (header, timestamp, id, etc.)
  bool isKeyField(const std::string & field_name) const;
  
  // Get type support for a message type
  const rosidl_message_type_support_t * getTypeSupportHandle(
    const std::string & type_name);
  
  // Load type support library dynamically
  void * loadTypeLibrary(const std::string & package_name);
  
  // Get symbol from library
  const rosidl_message_type_support_t * getTypeFromLibrary(
    void * lib_handle,
    const std::string & package_name,
    const std::string & msg_name);
  
  // Truncate output to max lines and chars per line
  std::string truncateOutput(
    const std::string & input,
    size_t max_lines = 200,
    size_t max_chars_per_line = 120) const;
  
  // Convert binary data to hex string (fallback)
  std::string toHexString(const uint8_t * data, size_t size, size_t max_bytes = 1024) const;
  
  // Create indentation string
  std::string indent(int level) const;
  
  // Cache for type support handles
  std::map<std::string, const rosidl_message_type_support_t *> type_support_cache_;
};

}  // namespace ros2_topic_monitor

#endif  // ROS2_TOPIC_MONITOR__MESSAGE_TYPE_LOADER_HPP_
