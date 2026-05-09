cmake_minimum_required(VERSION 3.8)
project(robotic_arm_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robotic_arm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_action)

add_executable(optical_server src/optical_server.cpp)
ament_target_dependencies(optical_server rclcpp robotic_arm_interfaces sensor_msgs cv_bridge)

add_executable(optical_action_server src/optical_action_server.cpp)
ament_target_dependencies(optical_action_server rclcpp rclcpp_action robotic_arm_interfaces moveit_ros_planning_interface)

add_executable(teleop src/teleop_key_action_client.cpp)
ament_target_dependencies(teleop rclcpp rclcpp_action robotic_arm_interfaces)

target_link_libraries(optical_server ${OpenCV_LIBRARIES})

install(TARGETS
  teleop
  optical_action_server
  optical_server
  DESTINATION lib/${PROJECT_NAME}/
)



ament_package()
