cmake_minimum_required(VERSION 2.8.3)
add_executable(test_pub_data
  test_pub_data.cpp
)
target_link_libraries(test_pub_data
   ${catkin_LIBRARIES}
)
