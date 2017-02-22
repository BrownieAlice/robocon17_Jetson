cmake_minimum_required(VERSION 2.8.3)
ADD_EXECUTABLE(test_pub_data
  test_pub_data.cpp
)
TARGET_LINK_LIBRARIES(test_pub_data
   ${catkin_LIBRARIES}
)
