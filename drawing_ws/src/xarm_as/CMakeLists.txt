cmake_minimum_required(VERSION 3.8)
project(xarm_as)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_common REQUIRED)
moveit_package()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  Boost
  control_msgs
  Eigen3
  interactive_markers
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
  rclcpp_action
  rosidl_default_generators
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  xarm_as_interfaces
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(action_server SHARED
  src/xarm_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "XARM_AS_BUILDING_DLL")
ament_target_dependencies(action_server
  "xarm_as_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(action_server PLUGIN "xarm_as::MoveArmActionServer" EXECUTABLE test_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  add_library(action_client SHARED
  src/xarm_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "XARM_AS_BUILDING_DLL")
ament_target_dependencies(action_client
  "xarm_as_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_node(action_client PLUGIN "xarm_as::main" EXECUTABLE test_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_executable(xarm_action_client src/xarm_action_client.cpp)
ament_target_dependencies(xarm_action_client
  "rclcpp"
  "rclcpp_action"
  "xarm_as_interfaces"
  "rclcpp_components"
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS xarm_action_client DESTINATION lib/${PROJECT_NAME})

add_executable(xarm_action_server src/xarm_action_server.cpp)
ament_target_dependencies(xarm_action_server
  "rclcpp"
  "rclcpp_action"
  "xarm_as_interfaces"
  "rclcpp_components"
  ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS xarm_action_server DESTINATION lib/${PROJECT_NAME})

# launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArm.action"
  DEPENDENCIES std_msgs geometry_msgs
)

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

ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
