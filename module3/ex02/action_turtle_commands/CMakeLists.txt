cmake_minimum_required(VERSION 3.8)
project(action_turtle_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(turtle_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)


add_library(turtle_action_server SHARED src/action_turtle_server.cpp)
target_include_directories(turtle_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(turtle_action_server
  PRIVATE "ACTION_TURTLE_COMMANDS_CPP_BUILDING_DLL")

ament_target_dependencies(turtle_action_server rclcpp rclcpp_action rclcpp_components turtle_action geometry_msgs std_msgs turtlesim)

rclcpp_components_register_node(turtle_action_server PLUGIN "turtle_action_cpp::ActionTurtleServer" EXECUTABLE action_turtle_server)
install(TARGETS
  turtle_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(action_turtle_client  src/action_turtle_client.cpp)

ament_target_dependencies(action_turtle_client rclcpp rclcpp_action rclcpp_components turtle_action geometry_msgs std_msgs turtlesim)

install(TARGETS
  action_turtle_client
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
