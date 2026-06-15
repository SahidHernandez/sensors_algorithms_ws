cmake_minimum_required(VERSION 3.8)
project(fisheye_camera)

find_package(ament_cmake REQUIRED)

# Instalar directorios de recursos
install(DIRECTORY
  launch
  config
  camera_info
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
