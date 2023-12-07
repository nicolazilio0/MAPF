cmake_minimum_required(VERSION 3.8)
project(send_borders)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(send_borders 
  src/send_borders.cpp
)
ament_target_dependencies(send_borders 
  rclcpp 
  geometry_msgs
  std_msgs
)

install(TARGETS
  send_borders
  DESTINATION lib/${PROJECT_NAME})

ament_package()
