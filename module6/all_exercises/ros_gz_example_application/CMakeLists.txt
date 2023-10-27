cmake_minimum_required(VERSION 3.5)

project(ros_gz_example_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(depth_follower depth_follower.cpp)
ament_target_dependencies(depth_follower rclcpp geometry_msgs sensor_msgs)

add_executable(lidar_follower lidar_follower.cpp)
ament_target_dependencies(lidar_follower rclcpp geometry_msgs sensor_msgs)


install(TARGETS
    lidar_follower
    depth_follower
    DESTINATION lib/${PROJECT_NAME})

ament_package()
