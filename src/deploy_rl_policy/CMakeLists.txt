cmake_minimum_required(VERSION 3.5)
project(deploy_rl_policy)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unitree_go REQUIRED)
# find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rosbag2_cpp REQUIRED)

include_directories(
  include
  )
link_directories(src)

add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
# add_executable(low_level_ctrl_simulation src/low_level_ctrl_simulation.cpp src/common/motor_crc.cpp)

install(PROGRAMS 
  scripts/rl_policy.py
  scripts/xbox_command.py
  scripts/wireless_controller_command.py
  scripts/config.py
  scripts/mujoco_simulator.py
  scripts/record_torque.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(low_level_ctrl
  rclcpp
  sensor_msgs
  unitree_go
  unitree_api
  std_msgs
)

install(TARGETS low_level_ctrl
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
