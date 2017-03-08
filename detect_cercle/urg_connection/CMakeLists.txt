cmake_minimum_required(VERSION 2.8.3)
add_executable(connect_usbLRF
  src/connect_usbLRF.cpp
  src/connect_LRF_lib.cpp
)
target_link_libraries(connect_usbLRF
   ${catkin_LIBRARIES}
   urg_c
)

add_executable(connect_ethLRF
  src/connect_ethLRF.cpp
  src/connect_LRF_lib.cpp
)
target_link_libraries(connect_ethLRF
   ${catkin_LIBRARIES}
   urg_c
)
