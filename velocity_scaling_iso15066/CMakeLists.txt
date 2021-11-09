cmake_minimum_required(VERSION 3.0.2)
project(velocity_scaling_iso15066)
add_compile_options(-std=c++11 -funroll-loops -Wall -O3 -Wall -mavx -mfma -ffast-math)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosdyn_core
  subscription_notifier
  geometry_msgs
  sensor_msgs
  tf
  tf_conversions
  moveit_ros_planning_interface
  name_sorting
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES #velocity_scaling_iso15066
  CATKIN_DEPENDS roscpp rosdyn_core subscription_notifier geometry_msgs sensor_msgs tf tf_conversions moveit_ros_planning_interface name_sorting
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
add_executable(test_node src/test.cpp )
add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

add_executable(unscaled_joint_target_pub
  src/unscaled_joint_target_pub.cpp
  src/${PROJECT_NAME}/microinterpolator.cpp)
add_dependencies(unscaled_joint_target_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(unscaled_joint_target_pub
  ${catkin_LIBRARIES}
)
