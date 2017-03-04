cmake_minimum_required(VERSION 2.8.3)
add_executable(disp_posi
  src/disp_posi.cpp
)
target_link_libraries(disp_posi
   ${catkin_LIBRARIES}
   ${GLUT_LIBRARY}
   ${OPENGL_LIBRARY}
   glpng
   m
)
