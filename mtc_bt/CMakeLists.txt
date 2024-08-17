cmake_minimum_required(VERSION 3.16)
project(mtc_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(THIS_PACKAGE_DEPS
    behaviortree_cpp
    behaviortree_ros2
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_task_constructor_core
    moveit_task_constructor_msgs
    rclcpp)

# ----------------------------------
add_library(
  mtc_plugin SHARED
  src/mtc_plugin.cpp
  src/mtc_init_task.cpp
  src/mtc_pipeline_planner.cpp
  src/mtc_cartesian_planner.cpp
  src/mtc_current_state.cpp
  src/mtc_move_to.cpp
  src/mtc_connect.cpp
  src/mtc_plan_task.cpp
  src/mtc_move_relative.cpp
  src/mtc_execute_task.cpp)
target_include_directories(
  mtc_plugin PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_compile_definitions(mtc_plugin PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(mtc_plugin ${THIS_PACKAGE_DEPS})

# Install headers
install(DIRECTORY include/ DESTINATION include)

# INSTALL plugins for other packages to load
install(
  TARGETS mtc_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
