cmake_minimum_required(VERSION 2.6)

project(hlds_3dtof)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-std=gnu++11")
set(CMAKE_CXX_FLAGS_DEBUG "-std=gnu++11 -g -O0")

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  tf
)

find_package(PkgConfig)
pkg_check_modules(LIBOPENCV2 REQUIRED opencv)

catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

include_directories(
  include
  ${LIBOPENCV2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(hlds_3dtof_node src/hlds_3dtof_node.cpp)

add_dependencies(
  hlds_3dtof_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  hlds_3dtof_node
  ${LIBOPENCV2_LDFLAGS}
  ${LIBOPENCV2_LIBRARIES}
  /usr/lib/hldstof/libtof.a
  rt m pthread stdc++
  ${catkin_LIBRARIES}
)