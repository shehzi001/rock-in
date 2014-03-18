cmake_minimum_required(VERSION 2.8.3)
project(arm_goal)

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

add_executable(arm_goal src/arm_goal.cpp)

target_link_libraries(arm_goal
   ${catkin_LIBRARIES}
)