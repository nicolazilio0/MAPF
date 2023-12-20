cmake_minimum_required(VERSION 3.5)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")


find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# CoordinateMapper
add_library(CoordinateMapper
  src/CoordinateMapper.cpp
)

# Map 
add_executable(env_map src/env_map.cpp)
ament_target_dependencies(env_map sensor_msgs cv_bridge image_transport geometry_msgs nav_msgs rclcpp std_msgs obstacles_msgs)
target_include_directories(env_map PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(env_map
  CoordinateMapper
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

# Voronoi
add_executable(voronoi_planner src/voronoi.cpp)
ament_target_dependencies(voronoi_planner geometry_msgs nav_msgs rclcpp std_msgs obstacles_msgs)
target_include_directories(voronoi_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(voronoi_planner
  CoordinateMapper
  Boost::headers
)


install(TARGETS
  env_map
  voronoi_planner
  CoordinateMapper
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()