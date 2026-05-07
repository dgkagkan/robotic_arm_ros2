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

add_executable(test_moveit src/test_moveit.cpp)
ament_target_dependencies(test_moveit rclcpp moveit_ros_planning_interface robotic_arm_interfaces)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(commander rclcpp moveit_ros_planning_interface example_interfaces robotic_arm_interfaces)

add_executable(optical_server src/optical_server.cpp)
ament_target_dependencies(optical_server rclcpp robotic_arm_interfaces sensor_msgs cv_bridge)

add_executable(optical_client src/optical_client.cpp)
ament_target_dependencies(optical_client rclcpp robotic_arm_interfaces moveit_ros_planning_interface)

add_executable(teleop_key src/teleop_key.cpp)
ament_target_dependencies(teleop_key rclcpp robotic_arm_interfaces)

add_executable(optical_action_server src/optical_action_server.cpp)
ament_target_dependencies(optical_action_server rclcpp rclcpp_action robotic_arm_interfaces moveit_ros_planning_interface)

target_link_libraries(optical_server ${OpenCV_LIBRARIES})

install(TARGETS
  optical_action_server
  teleop_key
  optical_server
  optical_client
  test_moveit
  commander
  DESTINATION lib/${PROJECT_NAME}/
)



ament_package()
