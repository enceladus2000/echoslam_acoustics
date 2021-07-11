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

# required for python nodes
ament_python_install_package(${PROJECT_NAME})

# install python scripts
set(python_scripts
  scripts/srv_compute_rir.py
  scripts/cli_compute_rir.py  
)
install(PROGRAMS
  ${python_scripts}
  DESTINATION lib/${PROJECT_NAME}
)


# install our custom interfaces
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  msg/Point3D.msg
  msg/Point3DArray.msg
  msg/FloatArray.msg
)
set(srv_files
  srv/ComputeRIR.srv
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
)

ament_package()
