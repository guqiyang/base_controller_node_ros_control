cmake_minimum_required(VERSION 2.8.3)
project(base_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  geometry_msgs
  hardware_interface
  controller_manager
#   ros_control
)

catkin_package(
  DEPENDS system_lib
  CATKIN_DEPENDS
  rospy
  std_msgs
  tf
  sensor_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(base_controller src/base_controller.cpp)
add_dependencies(base_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(base_controller
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
