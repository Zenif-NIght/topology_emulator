cmake_minimum_required(VERSION 2.8.3)
project(rebroadcaster_msgs)
find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
    ConnectPositionServer.srv
    ConnectTopics.srv
    DisconnectRebroadcast.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
)

