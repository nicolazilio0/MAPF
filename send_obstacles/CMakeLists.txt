cmake_minimum_required(VERSION 3.8)
project(send_obstacles)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(send_obstacles 
  src/send_obstacles.cpp
)
ament_target_dependencies(send_obstacles 
  rclcpp 
  geometry_msgs
  obstacles_msgs
  std_msgs
)

install(TARGETS
  send_obstacles
  DESTINATION lib/${PROJECT_NAME})

ament_package()
