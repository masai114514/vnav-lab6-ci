cmake_minimum_required(VERSION 2.8.3)
project(lab_6)

add_compile_options(-std=c++14)

# -g / --export-dynamic：崩溃时 ros 的 failure signal handler 才能打出函数名，
# 否则栈里全是 (unknown)。-O1 保留一点优化但不至于把变量优化掉。
add_compile_options(-g -O1)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")

find_package(lab_5 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  opencv3_catkin
  glog_catkin
  message_filters
  cv_bridge
  opengv
  eigen_catkin
  tf_conversions
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp rospy glog_catkin message_filters cv_bridge lab_5 opengv eigen_catkin opencv3_catkin
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(tracker_shim src/tracker_shim.cpp)
target_include_directories(tracker_shim PRIVATE ${lab_5_INCLUDE_DIRS})
target_link_libraries(tracker_shim PRIVATE ${lab_5_LIBRARIES})

add_executable(pose_estimation src/pose_estimation.cpp)
target_link_libraries(pose_estimation tracker_shim ${catkin_LIBRARIES})
