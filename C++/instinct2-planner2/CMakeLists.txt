cmake_minimum_required(VERSION 3.5)
project(planner2)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

add_executable(local_planner_node 
    src/local_planner.cpp
    src/histogram.cpp
    src/star_planner.cpp
    src/waypoint_generator.cpp
    src/pointcloud_processor.cpp
)

target_link_libraries(local_planner_node ${PCL_LIBRARIES})

target_include_directories(local_planner_node PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${pcl_conversions_INCLUDE_DIRS}
    include
)

ament_target_dependencies(local_planner_node
    rclcpp
    sensor_msgs
    px4_msgs
    geometry_msgs
    Eigen3
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    pcl_conversions
)

install(TARGETS
    local_planner_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()