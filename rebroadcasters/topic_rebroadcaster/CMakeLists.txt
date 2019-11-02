cmake_minimum_required(VERSION 2.8.3)
project(topic_rebroadcaster)

#add_compile_options(-std=c++11 -g -O0)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    rebroadcaster_msgs
    mv_msgs
    topic_tools
    roscpp
    std_msgs
)

find_package(GTest REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    rebroadcaster_msgs
    mv_msgs
    topic_tools
    roscpp
    std_msgs
  DEPENDS
    ${GTEST_LIBRARIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${GTEST_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Topic Server
add_executable(${PROJECT_NAME}_node
  src/server_node.cpp
  src/relay_server.cpp
  src/relay.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

## Demo
add_executable(${PROJECT_NAME}_demo test/demo.cpp)
set_target_properties(${PROJECT_NAME}_demo PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_demo PREFIX "")
add_dependencies(${PROJECT_NAME}_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_demo ${catkin_LIBRARIES})

## Tests
add_executable(test_relay_node test/test_relay.cpp src/relay.cpp)
set_target_properties(test_relay_node PROPERTIES OUTPUT_NAME test_relay_node PREFIX "")
add_dependencies(test_relay_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_relay_node ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_node

    ${PROJECT_NAME}_demo
    test_relay_node

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

