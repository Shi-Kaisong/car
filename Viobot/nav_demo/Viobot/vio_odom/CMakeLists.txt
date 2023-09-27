cmake_minimum_required(VERSION 2.8.3)
project(vio_odom)

add_compile_options(-std=c++14)

find_package(Eigen3)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  nav_robot
  pcl_conversions
  pcl_ros
  nodelet
)

catkin_package(
  CATKIN_DEPENDS 
  LIBRARIES vio_odom
  CATKIN_DEPENDS nodelet 
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(vio_odom src/vio_odom_nodelet.cpp)
target_link_libraries(vio_odom ${catkin_LIBRARIES})

add_executable(vio_odom_node
  src/vio_odom.cpp
)

add_dependencies(vio_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(vio_odom_node
  ${catkin_LIBRARIES}
)

install(TARGETS vio_odom vio_odom_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES vio_odom_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)