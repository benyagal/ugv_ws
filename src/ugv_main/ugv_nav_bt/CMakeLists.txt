cmake_minimum_required(VERSION 3.8)
project(ugv_nav_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

add_library(ugv_heading_calibration_bt_node SHARED src/heading_calibration.cpp)
target_compile_definitions(ugv_heading_calibration_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(ugv_heading_calibration_bt_node rclcpp std_msgs behaviortree_cpp_v3)

install(TARGETS ugv_heading_calibration_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})

ament_package()
