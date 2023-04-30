cmake_minimum_required(VERSION 3.0.2)
project(vuasrl_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
)

file(GLOB_RECURSE MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg ${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg)
add_message_files(FILES ${MSG_FILES})

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)  # Move this line before catkin_package()

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

