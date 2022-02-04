cmake_minimum_required(VERSION 3.0.2)
project(follow_joint_trajectory_echo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  subscription_notifier
  roscpp
  std_msgs
  actionlib
  moveit_planning_helper
  moveit_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #velocity_scaling_iso15066
  CATKIN_DEPENDS control_msgs moveit_planning_helper moveit_msgs subscription_notifier roscpp std_msgs actionlib
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
