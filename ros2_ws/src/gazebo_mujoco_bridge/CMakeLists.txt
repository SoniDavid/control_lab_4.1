cmake_minimum_required(VERSION 3.8)
project(gazebo_mujoco_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies (minimal required)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Install directories
install(DIRECTORY
  launch
  worlds
  urdf
  config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts (rename to drop .py so ros2 run finds them)
install(PROGRAMS scripts/trajectory_replay.py
  RENAME trajectory_replay
  DESTINATION lib/${PROJECT_NAME}
)

# Install mesh assets (STL files from the MuJoCo model)
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
