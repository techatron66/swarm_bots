cmake_minimum_required(VERSION 3.8)
project(dna_swarm_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/arduino_bridge.py
  ${PROJECT_NAME}/imu_odometry.py
  ${PROJECT_NAME}/marker_detector.py
  ${PROJECT_NAME}/swarm_coordinator.py
  ${PROJECT_NAME}/sim_detector.py
  ${PROJECT_NAME}/overtake_trigger.py
  scripts/generate_markers.py
  scripts/jetson_setup.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch worlds scripts models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
