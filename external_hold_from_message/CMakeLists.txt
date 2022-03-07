cmake_minimum_required(VERSION 3.0.2)
project(external_hold_from_message)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  subscription_notifier
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs subscription_notifier
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp
  )
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
