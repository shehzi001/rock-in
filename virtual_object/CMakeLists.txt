cmake_minimum_required(VERSION 2.8.3)
project(virtual_object)

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

add_executable(virtual_object src/virtual_object.cpp)

target_link_libraries(virtual_object
   ${catkin_LIBRARIES}
)