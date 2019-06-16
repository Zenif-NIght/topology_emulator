cmake_minimum_required(VERSION 2.8.3)
project(position_rebroadcaster)

add_compile_options(-std=c++11 -g -O0)

find_package(catkin REQUIRED
  COMPONENTS
    rebroadcaster_msgs
    mv_msgs
    roscpp
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
  INCLUDE_DIRS include
#  LIBRARIES position_rebroadcasters
  CATKIN_DEPENDS
    rebroadcaster_msgs
    mv_msgs
    roscpp
    message_runtime
    nav_msgs
  DEPENDS
    gtest
    gtest_main
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

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/position_rebroadcasters.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(agent_test test/test_agent.cpp src/agent.cpp)
set_target_properties(agent_test PROPERTIES OUTPUT_NAME agent_test PREFIX "")
add_dependencies(agent_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

add_executable(agent_pool_test test/test_agent_pool.cpp src/agent_pool.cpp src/agent.cpp)
set_target_properties(agent_pool_test PROPERTIES OUTPUT_NAME agent_pool_test PREFIX "")
add_dependencies(agent_pool_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(agent_pool_test ${GTEST_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS agent_test agent_pool_test
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
