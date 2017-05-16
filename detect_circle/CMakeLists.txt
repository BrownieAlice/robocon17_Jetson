cmake_minimum_required(VERSION 2.8.3)

project(detect_circle)

option(
  USE_CUDA_PROGRAM
  "Use cuda toolkit."
  ON
)
# cudaを持ちるか否か.
set(USE_CUDA_PROGRAM ON)

option(
  MAKE_ONLY_MSG
  "Make only msg."
  OFF
)
# msgファイルをincludeファイルに変換させるだけをやらせるか否か.
set(MAKE_ONLY_MSG OFF)

option(
  ROS_DEBUG_MODE
  "Debug mode."
  OFF
)
# デバックモードを使用する.
set(ROS_DEBUG_MODE OFF)

message(STATUS "USE_CUDA_PROGRAM ${USE_CUDA_PROGRAM}")
message(STATUS "MAKE_ONLY_MSG ${MAKE_ONLY_MSG}")
message(STATUS "ROS_DEBUG_MODE ${ROS_DEBUG_MODE}")

if(USE_CUDA_PROGRAM)
  find_package(CUDA REQUIRED)
endif(USE_CUDA_PROGRAM)
find_package(GLUT)
find_package(OpenGL)
find_package(OpenCV REQUIRED)
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
  Jcircle.msg
  Jline.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

roslint_cpp()

if(USE_CUDA_PROGRAM)
  set(CUDA_NVCC_FLAGS "-arch=sm_32" CACHE STRING "nvcc flags" FORCE)
  set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
  set(CUDA_PROPAGATE_HOST_FLAGS OFF)
endif(USE_CUDA_PROGRAM)
set(LIB_TYPE STATIC)
set(CMAKE_C_FLAGS "-Wall -O2 -march=native")
set(CMAKE_CXX_FLAGS "-D_FORCE_INLINES -Wall -O2 -march=native -std=c++11")

catkin_package(
  CATKIN_DEPENDS message_runtime
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

if(ROS_DEBUG_MODE)
  add_definitions(-DLINE_IO_DEBUG_MODE -DCIRCLE_IO_DEBUG_MODE)
endif(ROS_DEBUG_MODE)


if(NOT MAKE_ONLY_MSG)
  add_subdirectory(uart)

  add_subdirectory(test_pub_data)

  add_subdirectory(test_sub_data)

  if(USE_CUDA_PROGRAM)
    add_subdirectory(circle_io)
  endif(USE_CUDA_PROGRAM)

  add_subdirectory(disp_posi)

  add_subdirectory(urg_connection)

  add_subdirectory(line_io)
endif(NOT MAKE_ONLY_MSG)
