cmake_minimum_required(VERSION 2.8.3)
PROJECT (detect_cercle)
FIND_PACKAGE(CUDA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  message_generation
  roslint
)

add_message_files(
  FILES
  MBinput.msg
  Joutput.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

roslint_cpp()

SET(CUDA_NVCC_FLAGS "-arch=sm_32" CACHE STRING "nvcc flags" FORCE)
SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC)
SET(CUDA_PROPAGATE_HOST_FLAGS OFF)
SET(CMAKE_C_FLAGS "-Wall -O3")
# SET(CMAKE_CXX_FLAGS "-Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORCE_INLINES -Wall -O3 -std=c++11")

catkin_package(
  CATKIN_DEPENDS message_runtime
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(uart)

add_subdirectory(test_pub_data)

add_subdirectory(test_sub_data)

add_subdirectory(cercle_io)
