cmake_minimum_required(VERSION 3.8)
project(capra_landolt_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(capra_landolt_msgs REQUIRED)

add_executable(landolt_node src/landolt_node.cpp)

ament_target_dependencies(landolt_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  image_transport
  capra_landolt_msgs
)

target_link_libraries(landolt_node
  ${OpenCV_LIBRARIES}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()