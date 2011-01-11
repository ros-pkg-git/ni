cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE MinSizeRel)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include/kinect_base)

rosbuild_add_library(basicfreenect src/accel.c src/core.c src/tilt.c src/usb_libusb10.c)
target_link_libraries(basicfreenect usb-1.0)

rosbuild_add_executable(kinect_base_node src/kinect_base_node.cpp)
target_link_libraries(kinect_base_node basicfreenect)

rosbuild_add_executable(kinect_scan src/kinect_scan.cpp)
target_link_libraries(kinect_scan basicfreenect)
