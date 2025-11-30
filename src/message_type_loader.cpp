#include "ros2_topic_monitor/message_type_loader.hpp"
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_runtime_cpp/message_type_support_decl.hpp>
#include <rcpputils/shared_library.hpp>
#include <rcpputils/find_library.hpp>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstring>
#include <dlfcn.h>

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
  
  try {
    // Get type support handle
    const rosidl_message_type_support_t * type_support = getTypeSupportHandle(type_name);
    
    if (type_support) {
      // Get introspection type support
      const rosidl_message_type_support_t * introspection_ts = 
        get_message_typesupport_handle(
          type_support,
          rosidl_typesupport_introspection_cpp::typesupport_identifier);
      
      if (introspection_ts) {
        auto members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          introspection_ts->data);
        
        // Allocate memory for deserialized message
        std::vector<uint8_t> message_buffer(members->size_of_);
        void * message_data = message_buffer.data();
        
        // Initialize message
        if (members->init_function) {
          members->init_function(message_data, rosidl_runtime_cpp::MessageInitialization::ALL);
        }
        
        // Deserialize the message using type support
        try {
          // Create serializer with type support
          rclcpp::SerializationBase serializer(type_support);
          
          // Deserialize from serialized message
          serializer.deserialize_message(serialized_msg.get(), message_data);
          
          // Parse the deserialized message
          std::string parsed = parseMessage(message_data, members, 0);
          oss << parsed;
          
        } catch (const std::exception & e) {
          oss << "[Deserialization error: " << e.what() << "]\n";
        }
        
        // Cleanup
        if (members->fini_function) {
          members->fini_function(message_data);
        }
        
        return truncateOutput(oss.str());
      }
    }
  } catch (const std::exception & e) {
    oss << "[Parse error: " << e.what() << "]\n";
  }
  
  // Fallback to hex dump
  oss << "\n[Structured parsing failed, showing raw data]\n";
  const auto & rcl_msg = serialized_msg->get_rcl_serialized_message();
  std::string hex_dump = toHexString(
    rcl_msg.buffer,
    rcl_msg.buffer_length,
    1024
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

const rosidl_message_type_support_t * MessageTypeLoader::getTypeSupportHandle(
  const std::string & type_name)
{
  // Check cache first
  auto it = type_support_cache_.find(type_name);
  if (it != type_support_cache_.end()) {
    return it->second;
  }
  
  try {
    // Parse type name: "geometry_msgs/msg/PoseStamped" -> 
    // package: "geometry_msgs", type: "PoseStamped"
    size_t first_slash = type_name.find('/');
    size_t second_slash = type_name.find('/', first_slash + 1);
    
    if (first_slash == std::string::npos || second_slash == std::string::npos) {
      type_support_cache_[type_name] = nullptr;
      return nullptr;
    }
    
    std::string package_name = type_name.substr(0, first_slash);
    std::string msg_name = type_name.substr(second_slash + 1);
    
    // Load library
    void * lib_handle = loadTypeLibrary(package_name);
    if (!lib_handle) {
      type_support_cache_[type_name] = nullptr;
      return nullptr;
    }
    
    // Get type support
    const rosidl_message_type_support_t * ts = getTypeFromLibrary(
      lib_handle, package_name, msg_name);
    
    type_support_cache_[type_name] = ts;
    return ts;
  } catch (...) {
    type_support_cache_[type_name] = nullptr;
    return nullptr;
  }
}

void * MessageTypeLoader::loadTypeLibrary(const std::string & package_name)
{
  // Try multiple library name variations
  std::vector<std::string> library_names = {
    "lib" + package_name + "__rosidl_typesupport_cpp.so",
    "lib" + package_name + "__rosidl_typesupport_c.so",
    "lib" + package_name + "__rosidl_typesupport_introspection_cpp.so"
  };
  
  for (const auto & library_name : library_names) {
    void * lib_handle = dlopen(library_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
    if (lib_handle) {
      return lib_handle;
    }
  }
  
  return nullptr;
}

const rosidl_message_type_support_t * MessageTypeLoader::getTypeFromLibrary(
  void * lib_handle,
  const std::string & package_name,
  const std::string & msg_name)
{
  if (!lib_handle) {
    return nullptr;
  }
  
  // Try multiple symbol name formats
  std::vector<std::string> symbol_names = {
    // C++ introspection interface
    "rosidl_typesupport_introspection_cpp__get_message_type_support_handle__" + 
      package_name + "__msg__" + msg_name,
    // C++ interface  
    "rosidl_typesupport_cpp__get_message_type_support_handle__" + 
      package_name + "__msg__" + msg_name,
    // C interface
    "rosidl_typesupport_c__get_message_type_support_handle__" + 
      package_name + "__msg__" + msg_name
  };
  
  typedef const rosidl_message_type_support_t * (*get_type_support_func)();
  
  for (const auto & symbol_name : symbol_names) {
    get_type_support_func get_ts = reinterpret_cast<get_type_support_func>(
      dlsym(lib_handle, symbol_name.c_str()));
    
    if (get_ts) {
      const rosidl_message_type_support_t * ts = get_ts();
      if (ts) {
        return ts;
      }
    }
  }
  
  return nullptr;
}

std::string MessageTypeLoader::parseMessage(
  const void * message_data,
  const rosidl_typesupport_introspection_cpp::MessageMembers * members,
  int indent_level)
{
  if (!message_data || !members) {
    return "";
  }
  
  std::ostringstream oss;
  
  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const uint8_t * field_ptr = static_cast<const uint8_t *>(message_data) + member.offset_;
    
    std::string field_output = parseField(field_ptr, &member, indent_level);
    oss << field_output;
  }
  
  return oss.str();
}

std::string MessageTypeLoader::parseField(
  const void * field_data,
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  int indent_level)
{
  std::ostringstream oss;
  std::string field_name(member->name_);
  bool is_key = isKeyField(field_name);
  
  oss << indent(indent_level) << formatFieldName(field_name, is_key) << ": ";
  
  // Handle arrays
  if (member->is_array_) {
    oss << "\n" << parseArrayField(field_data, member, indent_level + 1);
    return oss.str();
  }
  
  // Handle different field types
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
  
  switch (member->type_id_) {
    case ROS_TYPE_BOOL:
      oss << (*static_cast<const bool *>(field_data) ? "true" : "false") << "\n";
      break;
    case ROS_TYPE_BYTE:
    case ROS_TYPE_UINT8:
      oss << static_cast<int>(*static_cast<const uint8_t *>(field_data)) << "\n";
      break;
    case ROS_TYPE_INT8:
      oss << static_cast<int>(*static_cast<const int8_t *>(field_data)) << "\n";
      break;
    case ROS_TYPE_UINT16:
      oss << *static_cast<const uint16_t *>(field_data) << "\n";
      break;
    case ROS_TYPE_INT16:
      oss << *static_cast<const int16_t *>(field_data) << "\n";
      break;
    case ROS_TYPE_UINT32:
      oss << *static_cast<const uint32_t *>(field_data) << "\n";
      break;
    case ROS_TYPE_INT32:
      oss << *static_cast<const int32_t *>(field_data) << "\n";
      break;
    case ROS_TYPE_UINT64:
      oss << *static_cast<const uint64_t *>(field_data) << "\n";
      break;
    case ROS_TYPE_INT64:
      oss << *static_cast<const int64_t *>(field_data) << "\n";
      break;
    case ROS_TYPE_FLOAT:
      oss << std::fixed << std::setprecision(6) << *static_cast<const float *>(field_data) << "\n";
      break;
    case ROS_TYPE_DOUBLE:
      oss << std::fixed << std::setprecision(6) << *static_cast<const double *>(field_data) << "\n";
      break;
    case ROS_TYPE_STRING:
      oss << "\"" << *static_cast<const std::string *>(field_data) << "\"\n";
      break;
    case ROS_TYPE_MESSAGE:
      if (member->members_) {
        oss << "\n";
        auto nested_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
          member->members_->data);
        oss << parseMessage(field_data, nested_members, indent_level + 1);
      } else {
        oss << "<nested message>\n";
      }
      break;
    default:
      oss << "<unknown type>\n";
      break;
  }
  
  return oss.str();
}

std::string MessageTypeLoader::parseArrayField(
  const void * field_data,
  const rosidl_typesupport_introspection_cpp::MessageMember * member,
  int indent_level)
{
  std::ostringstream oss;
  
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING;
  using rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE;
  
  // Get array size and data pointer
  size_t array_size = 0;
  const void * array_data = nullptr;
  
  if (member->array_size_ > 0 && !member->is_upper_bound_) {
    // Fixed size array
    array_size = member->array_size_;
    array_data = field_data;
  } else {
    // Dynamic array (std::vector)
    // For dynamic arrays, field_data points to a std::vector
    // We need to extract size and data from the vector
    if (member->type_id_ == ROS_TYPE_STRING) {
      auto vec = static_cast<const std::vector<std::string> *>(field_data);
      array_size = vec->size();
      array_data = vec->data();
    } else if (member->type_id_ == ROS_TYPE_MESSAGE) {
      // For message arrays, just show count
      oss << indent(indent_level) << "[Message array - display not implemented]\n";
      return oss.str();
    } else {
      // For primitive type vectors
      switch (member->type_id_) {
        case ROS_TYPE_UINT8:
        case ROS_TYPE_BYTE: {
          auto vec = static_cast<const std::vector<uint8_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_INT8: {
          auto vec = static_cast<const std::vector<int8_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_UINT16: {
          auto vec = static_cast<const std::vector<uint16_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_INT16: {
          auto vec = static_cast<const std::vector<int16_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_UINT32: {
          auto vec = static_cast<const std::vector<uint32_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_INT32: {
          auto vec = static_cast<const std::vector<int32_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_UINT64: {
          auto vec = static_cast<const std::vector<uint64_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_INT64: {
          auto vec = static_cast<const std::vector<int64_t> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_FLOAT: {
          auto vec = static_cast<const std::vector<float> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        case ROS_TYPE_DOUBLE: {
          auto vec = static_cast<const std::vector<double> *>(field_data);
          array_size = vec->size();
          array_data = vec->data();
          break;
        }
        default:
          oss << indent(indent_level) << "[Unknown array type]\n";
          return oss.str();
      }
    }
  }
  
  // Display array elements (limit to first 10 for readability)
  size_t max_display = std::min(array_size, static_cast<size_t>(10));
  
  if (member->type_id_ == ROS_TYPE_STRING) {
    const std::string * str_array = static_cast<const std::string *>(array_data);
    for (size_t i = 0; i < max_display; ++i) {
      oss << indent(indent_level) << "[" << i << "]: \"" << str_array[i] << "\"\n";
    }
  } else {
    // Format primitive types inline if small array
    if (array_size <= 5) {
      oss << indent(indent_level) << "[";
      for (size_t i = 0; i < array_size; ++i) {
        if (i > 0) oss << ", ";
        switch (member->type_id_) {
          case ROS_TYPE_UINT8:
          case ROS_TYPE_BYTE:
            oss << static_cast<int>(static_cast<const uint8_t *>(array_data)[i]);
            break;
          case ROS_TYPE_INT8:
            oss << static_cast<int>(static_cast<const int8_t *>(array_data)[i]);
            break;
          case ROS_TYPE_UINT16:
            oss << static_cast<const uint16_t *>(array_data)[i];
            break;
          case ROS_TYPE_INT16:
            oss << static_cast<const int16_t *>(array_data)[i];
            break;
          case ROS_TYPE_UINT32:
            oss << static_cast<const uint32_t *>(array_data)[i];
            break;
          case ROS_TYPE_INT32:
            oss << static_cast<const int32_t *>(array_data)[i];
            break;
          case ROS_TYPE_UINT64:
            oss << static_cast<const uint64_t *>(array_data)[i];
            break;
          case ROS_TYPE_INT64:
            oss << static_cast<const int64_t *>(array_data)[i];
            break;
          case ROS_TYPE_FLOAT:
            oss << std::fixed << std::setprecision(3) << static_cast<const float *>(array_data)[i];
            break;
          case ROS_TYPE_DOUBLE:
            oss << std::fixed << std::setprecision(3) << static_cast<const double *>(array_data)[i];
            break;
          default:
            oss << "?";
        }
      }
      oss << "]\n";
    } else {
      // Large array - show summary
      oss << indent(indent_level) << "[Array of " << array_size << " elements";
      if (array_size > max_display) {
        oss << ", showing first " << max_display;
      }
      oss << "]\n";
      
      for (size_t i = 0; i < max_display; ++i) {
        oss << indent(indent_level + 1) << "[" << i << "]: ";
        switch (member->type_id_) {
          case ROS_TYPE_UINT8:
          case ROS_TYPE_BYTE:
            oss << static_cast<int>(static_cast<const uint8_t *>(array_data)[i]);
            break;
          case ROS_TYPE_INT8:
            oss << static_cast<int>(static_cast<const int8_t *>(array_data)[i]);
            break;
          case ROS_TYPE_UINT16:
            oss << static_cast<const uint16_t *>(array_data)[i];
            break;
          case ROS_TYPE_INT16:
            oss << static_cast<const int16_t *>(array_data)[i];
            break;
          case ROS_TYPE_UINT32:
            oss << static_cast<const uint32_t *>(array_data)[i];
            break;
          case ROS_TYPE_INT32:
            oss << static_cast<const int32_t *>(array_data)[i];
            break;
          case ROS_TYPE_UINT64:
            oss << static_cast<const uint64_t *>(array_data)[i];
            break;
          case ROS_TYPE_INT64:
            oss << static_cast<const int64_t *>(array_data)[i];
            break;
          case ROS_TYPE_FLOAT:
            oss << std::fixed << std::setprecision(6) << static_cast<const float *>(array_data)[i];
            break;
          case ROS_TYPE_DOUBLE:
            oss << std::fixed << std::setprecision(6) << static_cast<const double *>(array_data)[i];
            break;
          default:
            oss << "<unknown>";
        }
        oss << "\n";
      }
      
      if (array_size > max_display) {
        oss << indent(indent_level + 1) << "... (" << (array_size - max_display) << " more elements)\n";
      }
    }
  }
  
  return oss.str();
}

std::string MessageTypeLoader::formatFieldName(
  const std::string & field_name,
  bool is_key_field) const
{
  if (is_key_field) {
    return "[*] " + field_name;  // Mark key fields
  }
  return field_name;
}

bool MessageTypeLoader::isKeyField(const std::string & field_name) const
{
  // List of important field names (exact match or specific patterns)
  static const std::vector<std::string> key_fields = {
    "header", "stamp", "timestamp", "frame_id", 
    "seq", "time"
  };
  
  // Fields that end with specific suffixes
  static const std::vector<std::string> key_suffixes = {
    "_id", "_name", "_time", "_stamp"
  };
  
  std::string lower_name = field_name;
  std::transform(lower_name.begin(), lower_name.end(), lower_name.begin(), ::tolower);
  
  // Exact match for common key fields
  for (const auto & key : key_fields) {
    if (lower_name == key) {
      return true;
    }
  }
  
  // Exact match for standalone "id" or "name"
  if (lower_name == "id" || lower_name == "name") {
    return true;
  }
  
  // Check for fields ending with key suffixes
  for (const auto & suffix : key_suffixes) {
    if (lower_name.size() >= suffix.size() && 
        lower_name.substr(lower_name.size() - suffix.size()) == suffix) {
      return true;
    }
  }
  
  return false;
}

std::string MessageTypeLoader::indent(int level) const
{
  return std::string(level * 2, ' ');
}

}  // namespace ros2_topic_monitor
