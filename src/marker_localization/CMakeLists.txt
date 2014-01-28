cmake_minimum_required(VERSION 2.8.3)
project(marker_localization)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp 
    dynamic_reconfigure
    sensor_msgs
    cv_bridge
    std_msgs
    message_generation
    message_filters
    tf

)

add_message_files(
  FILES
  Datamatrix.msg
)

generate_messages()

catkin_package(
CATKIN_DEPENDS message_runtime
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(agent_marker_localization_node
  src/agent_marker_localization_node.cpp

)

# make sure configure headers are built before any node using them
add_dependencies( agent_marker_localization_node ${PROJECT_NAME}_gencfg) 

target_link_libraries( agent_marker_localization_node
  ${catkin_LIBRARIES}
)
