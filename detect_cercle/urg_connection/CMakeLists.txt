cmake_minimum_required(VERSION 2.8.3)
add_executable(connect_usbLRF
  src/connect_usbLRF.cpp
)
target_link_libraries(connect_usbLRF
   ${catkin_LIBRARIES}
   urg_c
)
