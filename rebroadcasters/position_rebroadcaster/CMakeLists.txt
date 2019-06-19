cmake_minimum_required(VERSION 2.8.3)
project(position_rebroadcaster)

add_compile_options(-std=c++11 -g -O0)

find_package(catkin REQUIRED
  COMPONENTS
    rebroadcaster_msgs
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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    #  INCLUDE_DIRS include
#  LIBRARIES position_rebroadcasters
  CATKIN_DEPENDS
    rebroadcaster_msgs
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

## Specify additional locations of header files
## Your package locations should be listed before other locations
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

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
#catkin_add_gtest(agent_test test/test_agent.cpp src/agent.cpp)
#if(TARGET agent_test)
#  target_link_libraries(agent_test
#    gtest
#    gtest_main
#    ${catkin_LIBRARIES}
#  )
#endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
