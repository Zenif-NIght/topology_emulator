cmake_minimum_required(VERSION 2.8.3)
project(mv_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    message_generation
    message_runtime
    geometry_msgs
    std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    Neighbors.msg
    VehiclePose.msg
    VehiclePoses.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)

