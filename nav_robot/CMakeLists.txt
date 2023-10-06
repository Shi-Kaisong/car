cmake_minimum_required(VERSION 3.0.2)
project(nav_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_base
  roscpp
  std_msgs
  actionlib 
  nav_msgs
  actionlib_msgs
  message_generation
)
find_package(Eigen3)

add_message_files(
  FILES
  Goal.msg
  GoalArray.msg
)

add_service_files(
  FILES
  GetNavState.srv
  NavGoalCmd.srv
)

add_action_files(
  FILES
  goal_send.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

