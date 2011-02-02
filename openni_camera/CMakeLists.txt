cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})
#set(CMAKE_CXX_FLAGS "-O0 -g -pg" )
#add_definitions ( -O0 -g )
add_definitions (-Wno-missing-field-initializers -O4 -DNDEBUG -mfpmath=sse -msse4 -fopenmp)
# Dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# ---[ OpenNI Driver
rosbuild_add_library (openni_driver src/openni_driver.cpp
                                    src/openni_device.cpp
                                    src/openni_exception.cpp
                                    src/openni_device_primesense.cpp
                                    src/openni_device_kinect.cpp
                                    src/openni_image_yuv_422.cpp
                                    src/openni_image_bayer_grbg.cpp
                                    src/openni_depth_image.cpp
                     )
rosbuild_add_compile_flags (openni_driver ${SSE_FLAGS})

rosbuild_add_library (openni_nodelet src/openni_nodelet.cpp)
rosbuild_add_compile_flags (openni_nodelet ${SSE_FLAGS})

rosbuild_add_executable (openni_example src/openni_example.cpp)
target_link_libraries (openni_example openni_driver usb)

rosbuild_add_executable (openni_node src/openni_node.cpp)
target_link_libraries (openni_node openni_driver usb)
