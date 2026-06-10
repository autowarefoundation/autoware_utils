cmake_minimum_required(VERSION 3.14)
project(autoware_utils_tf)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${test_files})
  target_include_directories(test_${PROJECT_NAME} PUBLIC ${autoware_utils_geometry_INCLUDE_DIRS})
  target_link_libraries(test_${PROJECT_NAME}
    rclcpp::rclcpp ${geometry_msgs_TARGETS} tf2_ros::tf2_ros ${autoware_utils_geometry_LIBRARIES}
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE include)
endif()

ament_auto_package()
