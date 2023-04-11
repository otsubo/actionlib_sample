cmake_minimum_required(VERSION 2.8.3)

project(actionlib_sample)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  )

#add_message_files(
#  )

add_action_files(
  FILES
  SimpleControl.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  )


install(
  DIRECTORY DESTINATION node_scripts
)
