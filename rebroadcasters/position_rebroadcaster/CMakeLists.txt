cmake_minimum_required(VERSION 2.8.3)
project(position_rebroadcaster)

add_compile_options(-std=c++11 -g -O0)
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    rebroadcaster_msgs
    network_topology_emulator
    mv_msgs
    roscpp
    tf
    message_runtime
    nav_msgs
)

find_package(GTest REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    rebroadcaster_msgs
    network_topology_emulator
    mv_msgs
    roscpp
    tf
    nav_msgs
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

## Position Rebroadcaster Node
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/output_server.cpp
  src/position_publisher.cpp
  src/agent_pool.cpp
  src/agent.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_node PREFIX "")
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

## Demo
add_executable(${PROJECT_NAME}_demo test/full_demo.cpp)
set_target_properties(${PROJECT_NAME}_demo PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_demo PREFIX "")
add_dependencies(${PROJECT_NAME}_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_demo ${catkin_LIBRARIES})

## Tests
add_executable(agent_test test/test_agent.cpp src/agent.cpp)
set_target_properties(agent_test PROPERTIES OUTPUT_NAME agent_test PREFIX "")
add_dependencies(agent_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(agent_pool_test test/test_agent_pool.cpp src/agent_pool.cpp src/agent.cpp)
set_target_properties(agent_pool_test PROPERTIES OUTPUT_NAME agent_pool_test PREFIX "")
add_dependencies(agent_pool_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_pool_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(position_publisher_test test/test_position_publisher.cpp src/position_publisher.cpp src/agent_pool.cpp src/agent.cpp)
set_target_properties(position_publisher_test PROPERTIES OUTPUT_NAME position_publisher_test PREFIX "")
add_dependencies(position_publisher_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_publisher_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_node

    ${PROJECT_NAME}_demo
    agent_test
    agent_pool_test
    position_publisher_test

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

