cmake_minimum_required(VERSION 2.8.3)
project(skiros_resource)

set(CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  pluginlib
  roscpp
  skiros_common
  skiros_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES skiros_device
#  CATKIN_DEPENDS actionlib pluginlib roscpp skiros_common skiros_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

# find and setup Qt4 for this project
set(QT_QMAKE_EXECUTABLE /usr/bin/qmake-qt4) #Force the system to use QT4
find_package(Qt4 REQUIRED)

SET(QT_USE_QTXML true)
SET(QT_USE_QTNETWORK true)
SET(QT_USE_QTGUI true)
SET(QT_USE_QTCORE true)
SET(QT_USE_QDBUS true)

include(${QT_USE_FILE})


## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_library(skiros_resource src/gripper_action_server.cpp)
## Declare a cpp executable
add_executable(resource_manager_node src/resource_manager.cpp src/resource_manager_node.cpp)
add_executable(test_node_spawn src/test_spawn.cpp)
add_executable(test_proxy src/test_proxy.cpp)
add_executable(device_node src/standalone_device_node.cpp)

target_link_libraries(skiros_resource ${catkin_LIBRARIES})
target_link_libraries(resource_manager_node skiros_resource ${catkin_LIBRARIES})
target_link_libraries(test_node_spawn ${catkin_LIBRARIES})
target_link_libraries(test_proxy ${catkin_LIBRARIES})
target_link_libraries(device_node  skiros_resource ${catkin_LIBRARIES})


add_dependencies(resource_manager_node skiros_msgs_generate_messages_cpp)
add_dependencies(test_proxy skiros_msgs_generate_messages_cpp)
add_dependencies(device_node skiros_msgs_generate_messages_cpp)


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
# install(TARGETS skiros_device skiros_device_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_skiros_device.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
