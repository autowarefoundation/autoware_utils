cmake_minimum_required(VERSION 3.14)
project(autoware_utils_diagnostics)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/diagnostics_interface.cpp"
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    "test/main.cpp"
    "test/cases/diagnostics_interface.cpp"
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package()
