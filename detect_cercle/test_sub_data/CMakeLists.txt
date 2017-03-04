cmake_minimum_required(VERSION 2.8.3)
add_executable(test_sub_data
  test_sub_data.cpp
)
target_link_libraries(test_sub_data
   ${catkin_LIBRARIES}
)
