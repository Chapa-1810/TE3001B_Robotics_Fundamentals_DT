cmake_minimum_required(VERSION 3.8)
project(main_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(figure_msgs REQUIRED)
find_package(main_interfaces REQUIRED)

add_executable(main_engine src/main_engine.cpp)
ament_target_dependencies(main_engine main_interfaces rclcpp rclcpp_action std_msgs) # add main_interfaces

install(TARGETS
  main_engine
  DESTINATION lib/${PROJECT_NAME})


ament_package()
