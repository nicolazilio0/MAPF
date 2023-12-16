cmake_minimum_required(VERSION 3.5)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(env_map src/env_map.cpp)
add_executable(voronoi_planner src/voronoi.cpp)

ament_target_dependencies(env_map geometry_msgs nav_msgs rclcpp std_msgs obstacles_msgs)

target_link_libraries(env_map Boost::headers)
target_link_libraries(env_map Eigen3::Eigen)
target_link_libraries(env_map ${OpenCV_LIBRARIES})

ament_target_dependencies(voronoi_planner geometry_msgs nav_msgs rclcpp std_msgs obstacles_msgs)

target_link_libraries(voronoi_planner Boost::headers)


install(TARGETS
  env_map
  voronoi_planner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()