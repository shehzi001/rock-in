cmake_minimum_required(VERSION 2.8.3)
project(goto_datamatrix)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(goto_datamatrix src/goto_datamatrix.cpp)

target_link_libraries(goto_datamatrix
   ${catkin_LIBRARIES}
)