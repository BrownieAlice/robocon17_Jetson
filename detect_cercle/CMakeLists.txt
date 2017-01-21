cmake_minimum_required(VERSION 2.8.3)
PROJECT (detect_cercle)
FIND_PACKAGE(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

SET(CUDA_NVCC_FLAGS "-arch=sm_32" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC) 

catkin_package(
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

CUDA_ADD_LIBRARY(DETECT_CERCLE ${LIB_TYPE} src/detect_cercle_cuda.cu)
ADD_EXECUTABLE(detect_cercle src/detect_cercle.cpp)
TARGET_LINK_LIBRARIES(detect_cercle
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   DETECT_CERCLE
)
