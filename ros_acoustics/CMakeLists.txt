cmake_minimum_required(VERSION 3.5)
project(ros_acoustics)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# no build testing yet

# install our custom interfaces
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Point3D.msg
  msg/Point3DArray.msg
)
set(srv_files
  # srv/AddThreeInts.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  # ${srv_files}
)

ament_package()
