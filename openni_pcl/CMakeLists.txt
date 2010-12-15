cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include ($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
add_definitions (-Wall -O3 -pipe -funroll-loops)
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

rosbuild_add_executable (openni_viewer src/openni_viewer.cpp)
rosbuild_add_compile_flags (openni_viewer ${SSE_FLAGS})

#rosbuild_add_library (openni_pcl_nodelets src/openni_viewer_nodelet.cpp)
#rosbuild_add_compile_flags (openni_pcl_nodelets ${SSE_FLAGS})

