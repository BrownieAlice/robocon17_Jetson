cmake_minimum_required(VERSION 2.8.3)
cuda_add_library(CUDA_DETECT ${LIB_TYPE} src/detect_circle_cuda.cu)
add_executable(circle_io
  src/circle_io.cpp
  src/pole.cpp
  src/write_circle.cpp
)
target_link_libraries(circle_io
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   CUDA_DETECT
)
