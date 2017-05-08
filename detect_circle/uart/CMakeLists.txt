cmake_minimum_required(VERSION 2.8.3)
add_subdirectory(lib)
ADD_EXECUTABLE(MB_uart_communication
  MB_uart_communication.cpp
)
TARGET_LINK_LIBRARIES(MB_uart_communication
   ${catkin_LIBRARIES}
   UART_LIB
)
