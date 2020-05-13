cmake_minimum_required(VERSION 3.5)
project(ecard)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_cmake
find_package(ament_cmake REQUIRED)
# ROS2 packages
find_package(rclcpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# ROS2 interfaces
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geometric_primitive_msgs REQUIRED)

# Eigen
find_package(Eigen3 REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# Executables
set(EXECUTABLE_0 ecard)

# Executable 0
add_executable(${EXECUTABLE_0} src/${EXECUTABLE_0}.cpp)
target_link_libraries(${EXECUTABLE_0} ${EIGEN3_LIBRARIES})
ament_target_dependencies(
  ${EXECUTABLE_0}
  rclcpp
  moveit_ros_planning_interface
  tf2
  tf2_ros
  std_srvs
  geometry_msgs
  geometric_primitive_msgs
  Boost)

# Install targets
install(TARGETS ${EXECUTABLE_0} DESTINATION lib/${PROJECT_NAME})
# Install directories
install(DIRECTORY config launch users DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
