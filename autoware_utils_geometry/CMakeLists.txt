cmake_minimum_required(VERSION 3.14)
project(autoware_utils_geometry)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/geometry/alt_geometry.cpp"
  "src/geometry/boost_polygon_utils.cpp"
  "src/geometry/ear_clipping.cpp"
  "src/geometry/geometry.cpp"
  "src/geometry/gjk_2d.cpp"
  "src/geometry/pose_deviation.cpp"
  "src/geometry/random_concave_polygon.cpp"
  "src/geometry/random_convex_polygon.cpp"
  "src/geometry/sat_2d.cpp"
  "src/msg/operation.cpp"
)

if(BUILD_TESTING)
#   file(GLOB_RECURSE test_files test/*.cpp)
#   file(GLOB_RECURSE test_files test/cases/alt_geometry.cpp)
  file(GLOB_RECURSE test_files 
    # test/cases/alt_geometry.cpp
    # test/cases/boost_geometry.cpp
    test/cases/boost_polygon_utils.cpp
    test/cases/pose_deviation.cpp
    test/cases/path_with_lane_id_geometry.cpp
    test/cases/geometry.cpp
    )

  ament_auto_add_gtest(test_${PROJECT_NAME} ${test_files})
endif()

ament_auto_package()
