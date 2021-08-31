cmake_minimum_required(VERSION 3.5)
project(ros2_costmap_to_dynamic_obstacles)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_dynamic_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


set(dependencies
    unique_identifier_msgs
    nav2_dynamic_msgs
    cv_bridge
    geometry_msgs
    nav2_costmap_2d
    tf2
    tf2_geometry_msgs
    tf2_ros
    OpenCV
    rclcpp
    rclcpp_lifecycle
)

include_directories(
  include
)

add_library(ros2_costmap_to_dynamic_obstacles SHARED
  src/background_subtractor.cpp
  src/blob_detector.cpp
  src/costmap_to_dynamic_obstalces.cpp
)

ament_target_dependencies(ros2_costmap_to_dynamic_obstacles 
    ${dependencies}
)

# Remember when running the converter node through ros2 run, it will be called: "my_converter_node"
add_executable(my_converter_node src/costmap_converter_node.cpp)
target_link_libraries(my_converter_node
ros2_costmap_to_dynamic_obstacles
)
ament_target_dependencies(my_converter_node
  ${dependencies}
)


install(TARGETS ros2_costmap_to_dynamic_obstacles
   DESTINATION lib/${PROJECT_NAME} 
)
install(TARGETS my_converter_node
   DESTINATION lib/${PROJECT_NAME}      # changed from DESTINATION bin
)
install(DIRECTORY include/
   DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(ros2_costmap_to_dynamic_obstacles)
ament_export_dependencies(${dependencies})


ament_package()
