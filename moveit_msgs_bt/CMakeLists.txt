cmake_minimum_required(VERSION 3.16)
project(moveit_msgs_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_cpp
    behaviortree_ros2
    moveit_msgs
    rclcpp)

# ----------------------------------
add_library(
  moveit_msgs_plugin SHARED
  src/moveit_msgs_plugin.cpp
  src/msg/create_collision_object.cpp)
target_include_directories(
  moveit_msgs_plugin PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_compile_definitions(moveit_msgs_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(moveit_msgs_plugin ${THIS_PACKAGE_DEPS})

# Install headers
install(DIRECTORY include/ DESTINATION include)

# INSTALL plugins for other packages to load
install(
  TARGETS moveit_msgs_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
