cmake_minimum_required(VERSION 3.15)
project(matlab_bridge)

# default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(control_libraries 7.0.0 REQUIRED)
find_package(network_interfaces 1.2.0 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

ament_auto_add_executable(matlab_bridge src/MatlabBridge.cpp src/FrankaInterface.cpp)
target_link_libraries(matlab_bridge ${control_libraries_LIBRARIES} ${network_interfaces_LIBRARIES})

ament_auto_add_executable(print_robot_state src/RobotStateSubscriber.cpp)
ament_auto_add_executable(obstacle_bridge src/ObstaclePublisher.cpp)

install(TARGETS
  matlab_bridge
  print_robot_state
  obstacle_bridge
  DESTINATION lib/${PROJECT_NAME})

# Install other necessary files
install(
  PROGRAMS
  scripts/ObstacleZMQBridge.py
  scripts/PathZMQBridge.py
  scripts/ObstacleStreamerBridge.py
  scripts/MppiController.py
  scripts/MppiPlanner.py
  scripts/ObstacleManagerGUI.py
  DESTINATION lib/${PROJECT_NAME}
)  

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_auto_package()
