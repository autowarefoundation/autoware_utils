cmake_minimum_required(VERSION 3.14)
project(autoware_utils_system)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/backtrace.cpp"
)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_auto_add_gtest(test_${PROJECT_NAME} ${test_files})
endif()

ament_auto_package()
