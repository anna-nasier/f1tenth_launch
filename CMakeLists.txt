cmake_minimum_required(VERSION 3.14)
project(f1tenth_launch)

find_package(ament_cmake_auto REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/obstacle_avoidance.py scripts/obstacle_detection.py
  DESTINATION lib/${PROJECT_NAME}/
)

ament_auto_package(INSTALL_TO_SHARE
  scripts
  launch
  rviz
  config
)



