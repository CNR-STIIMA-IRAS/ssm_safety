cmake_minimum_required(VERSION 3.0.2)
project(velocity_scaling_multi_robot)

find_package(catkin REQUIRED COMPONENTS
  robot_state_controller_msgs
  roscpp
  velocity_scaling_iso15066
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #velocity_scaling_iso15066
  CATKIN_DEPENDS   robot_state_controller_msgs roscpp velocity_scaling_iso15066
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp
  )
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
