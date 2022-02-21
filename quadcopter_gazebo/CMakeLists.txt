cmake_minimum_required(VERSION 2.8.8)
project(quadcopter)
FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
add_subdirectory(msg)
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msg
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msg)


add_message_files(
  FILES
  SendActuator.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable (speed_publisher src/publish_actuators.cpp)
target_link_libraries(speed_publisher command_motor_speed_msgs ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(speed_publisher command_motor_speed_msgs quadcopter_generate_messages_cpp)

