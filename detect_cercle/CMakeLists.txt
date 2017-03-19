cmake_minimum_required(VERSION 2.8.3)

project(detect_cercle)

find_package(CUDA REQUIRED)
find_package(GLUT)
find_package(OpenGL)
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

set(CUDA_NVCC_FLAGS "-arch=sm_32" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
set(LIB_TYPE STATIC)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)
set(CMAKE_C_FLAGS "-Wall -O2 -march=native")
set(CMAKE_CXX_FLAGS "-D_FORCE_INLINES -Wall -O2 -march=native -std=c++14")

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

add_subdirectory(disp_posi)

add_subdirectory(urg_connection)
