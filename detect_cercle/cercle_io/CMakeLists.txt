cmake_minimum_required(VERSION 2.8.3)
CUDA_ADD_LIBRARY(CUDA_DETECT ${LIB_TYPE} src/detect_cercle_cuda.cu)
ADD_EXECUTABLE(cercle_io
  src/cercle_io.cpp
  src/pole.cpp
)
TARGET_LINK_LIBRARIES(cercle_io
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   CUDA_DETECT
)
