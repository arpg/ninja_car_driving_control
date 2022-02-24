cmake_minimum_required(VERSION 3.0.2)
project(drive_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
)

add_message_files(
  DIRECTORY
  msg
  FILES
  DriveCommand.msg
  )

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_drive_joy_node 
  src/teleop_drive_joy_node.cpp
  src/teleop_drive_joy.cpp
  )
target_link_libraries(teleop_drive_joy_node
  ${catkin_LIBRARIES}
  )
