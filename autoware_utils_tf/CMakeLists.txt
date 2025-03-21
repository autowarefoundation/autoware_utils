cmake_minimum_required(VERSION 3.14)
project(autoware_utils_tf)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${test_files})
  ament_target_dependencies(test_${PROJECT_NAME}
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
    ${${PROJECT_NAME}_FOUND_TEST_DEPENDS}
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE include)
endif()

ament_auto_package()
