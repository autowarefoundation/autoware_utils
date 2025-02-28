cmake_minimum_required(VERSION 3.14)
project(autoware_utils_pcl)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
endif()

ament_auto_package()
