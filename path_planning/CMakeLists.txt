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
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# DubinsPath
add_library(DubinsPath
  src/DubinsPath.cpp
)

# RRTStarDubins
add_library(RRTStarDubins
  src/RRTStarDubins.cpp
)

# RRTStarDubins
add_library(VoronoiDijkstra
  src/VoronoiDijkstra.cpp
)

target_include_directories(DubinsPath PRIVATE ${Eigen3_INCLUDE_DIRS})
target_link_libraries(DubinsPath Eigen3::Eigen)

# Map 
add_executable(env_map src/env_map.cpp)
ament_target_dependencies(env_map sensor_msgs cv_bridge image_transport geometry_msgs nav_msgs rclcpp std_msgs obstacles_msgs)
target_include_directories(env_map PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(env_map
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
  Boost::headers
)

# RRT*
add_executable(rrt_star_planner src/rrt_star.cpp)
ament_target_dependencies(rrt_star_planner geometry_msgs nav_msgs rclcpp std_msgs obstacles_msgs)

target_include_directories(rrt_star_planner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(rrt_star_planner
  DubinsPath
  RRTStarDubins
  Eigen3::Eigen
)

# Orchestrator
add_executable(orchestrator src/orchestrator.cpp)
ament_target_dependencies(orchestrator geometry_msgs nav_msgs  nav2_msgs rclcpp rclcpp_action std_msgs)

target_include_directories(orchestrator PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS
  DubinsPath
  RRTStarDubins
  env_map
  voronoi_planner
  rrt_star_planner
  orchestrator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()