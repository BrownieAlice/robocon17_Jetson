cmake_minimum_required(VERSION 2.8.3)
add_executable(disp_situation
  disp_situation.cpp
)
target_link_libraries(disp_situation
   ${catkin_LIBRARIES}
)
