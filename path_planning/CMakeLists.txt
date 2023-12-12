cmake_minimum_required(VERSION 3.5)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(voronoi_map src/voronoi_map.cpp)
ament_target_dependencies(voronoi_map geometry_msgs rclcpp std_msgs obstacles_msgs)

target_link_libraries(voronoi_map Boost::headers)
target_link_libraries(voronoi_map Eigen3::Eigen)
target_link_libraries(voronoi_map ${OpenCV_LIBRARIES})


install(TARGETS
voronoi_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()