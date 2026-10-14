cmake_minimum_required(VERSION 3.8)
project(ugv_costmap_layers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(ugv_costmap_layers SHARED src/chicken_layer.cpp)
ament_target_dependencies(ugv_costmap_layers
  rclcpp nav2_costmap_2d pluginlib geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

pluginlib_export_plugin_description_file(nav2_costmap_2d chicken_layer_plugin.xml)

install(TARGETS ugv_costmap_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
