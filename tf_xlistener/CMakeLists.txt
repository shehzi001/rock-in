cmake_minimum_required(VERSION 2.8.3)
project(tf_xlistener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)

catkin_package(
#INCLUDE_DIRS include
#  LIBRARIES tracker_listener
#  CATKIN_DEPENDS roscpp std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(tf_xlistener src/tf_xlistener.cpp)

#target_link_libraries(tf_xlistener
  # ${catkin_LIBRARIES}
#)