cmake_minimum_required(VERSION 3.14)
project(autoware_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
