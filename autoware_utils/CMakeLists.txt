cmake_minimum_required(VERSION 3.14)
project(autoware_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Boost REQUIRED)
target_compile_definitions(autoware_utils
  PRIVATE BOOST_MAJOR_VERSION=${Boost_MAJOR_VERSION} BOOST_MINOR_VERSION=${Boost_MINOR_VERSION})

file(GLOB_RECURSE src_files
    src/*.cpp
    src/geometry/*.cpp
    src/math/*.cpp
    src/ros/*.cpp
    src/system/*.cpp
)

ament_auto_add_library(autoware_utils SHARED
  ${src_files}
)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/**/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_utils ${test_files})
  target_link_libraries(test_autoware_utils
    autoware_utils
  )
endif()

ament_auto_package()
