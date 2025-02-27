cmake_minimum_required(VERSION 3.14)
project(autoware_utils_math)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/sin_table.cpp"
  "src/trigonometry.cpp"
)

if(BUILD_TESTING)
endif()

ament_auto_package()
