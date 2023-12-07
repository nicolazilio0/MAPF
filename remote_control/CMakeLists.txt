cmake_minimum_required(VERSION 3.8)
project(remote_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-w)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

add_executable(remote_control
  src/remote_control.cpp
)

add_executable(joystick_control
  src/joystick_control.cpp
  src/joystick.cc
)

ament_target_dependencies(remote_control 
  rclcpp
  geometry_msgs
  std_srvs
)

ament_target_dependencies(joystick_control 
  rclcpp
  geometry_msgs
  std_srvs
)

target_link_libraries(remote_control
  ${rclcpp_LIBRARIES} -lncurses -pthread
)

target_link_libraries(joystick_control
  ${rclcpp_LIBRARIES} -pthread
)

install(TARGETS
  remote_control
  joystick_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
