cmake_minimum_required(VERSION 2.8.3)
ADD_EXECUTABLE(test_sub_data
  test_sub_data.cpp
)
TARGET_LINK_LIBRARIES(test_sub_data
   ${catkin_LIBRARIES}
)
