cmake_minimum_required(VERSION 3.8)
project(ros2_topic_monitor)

# Set C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Find ncurses
find_package(Curses REQUIRED)

# Include directories
include_directories(
  include
  ${CURSES_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/main.cpp
  src/topic_discovery.cpp
  src/lazy_subscriber.cpp
  src/metrics_manager.cpp
  src/message_type_loader.cpp
  src/ui_main.cpp
  src/ui_detail.cpp
)

# Executable
add_executable(monitor ${SOURCES})

ament_target_dependencies(monitor
  rclcpp
  std_msgs
  sensor_msgs
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  rcl_interfaces
)

target_link_libraries(monitor
  ${CURSES_LIBRARIES}
  pthread
)

# Install
install(TARGETS
  monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
