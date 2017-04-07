cmake_minimum_required(VERSION 2.8.3)
add_executable(line_io
  src/line_io.cpp
  src/write_line.cpp
  lib/lsd.c
)
target_link_libraries(line_io
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
include_directories(
   ${OpenCV_INCLUDE_DIRS}
 )
