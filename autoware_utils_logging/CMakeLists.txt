cmake_minimum_required(VERSION 3.14)
project(autoware_utils_logging)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    "test/main.cpp"
    "test/cases/logger_level_configure.cpp"
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE include)
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp logging_demo)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
