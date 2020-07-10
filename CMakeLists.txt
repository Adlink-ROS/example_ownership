cmake_minimum_required(VERSION 3.5)
project(nsdk_ownership)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_VERBOSE_MAKEFILE ON)
#set(CMAKE_BUILD_TYPE Debug)

# Add debug definitions
if(UNIX)
  add_definitions("-DLINUX -O2 -s -Wall ")
endif(UNIX)


# Find Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(cyclonedds_options REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Ownership.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  ADD_LINTER_TESTS
  )

# Source Code
add_executable(ownership_pub src/publisher.cpp)
ament_target_dependencies(ownership_pub
  rclcpp
  rmw
  cyclonedds_options)
rosidl_target_interfaces(ownership_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(ownership_sub src/subscriber.cpp)
ament_target_dependencies(ownership_sub
  rclcpp
  rmw 
  cyclonedds_options)
rosidl_target_interfaces(ownership_sub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  ownership_pub
  ownership_sub
  DESTINATION lib/${PROJECT_NAME}
  )

ament_export_dependencies(rosidl_default_runtime)

ament_package()