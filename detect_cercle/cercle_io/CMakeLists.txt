cmake_minimum_required(VERSION 2.8.3)
cuda_add_library(CUDA_DETECT ${LIB_TYPE} src/detect_cercle_cuda.cu)
add_executable(cercle_io
  src/cercle_io.cpp
  src/pole.cpp
  src/write_cercle.cpp
)
target_link_libraries(cercle_io
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   CUDA_DETECT
)
