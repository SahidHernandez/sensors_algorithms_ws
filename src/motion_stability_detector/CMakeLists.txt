cmake_minimum_required(VERSION 3.8)
project(motion_stability_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencias
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Instalar scripts Python
install(
  PROGRAMS
    scripts/motion_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instalar launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Instalar configs
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()