cmake_minimum_required(VERSION 2.8.3)
project(skiros_skill)

set(CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  actionlib
  roscpp
  pluginlib
  skiros_msgs
  skiros_common
  skiros_config
  skiros_world_model
)

## System dependencies are found with CMake's conventions (skill-AAU)
find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES skill_base
  CATKIN_DEPENDS actionlib roscpp pluginlib skiros_msgs skiros_common skiros_world_model
#  DEPENDS system_lib
)

#catkin_python_setup()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
 add_library(skill_base
   src/skill_base.cpp
   src/skill_manager_interface.cpp
   src/skill_layer_interface.cpp
   src/modules_interface.cpp
   src/module_monitor.cpp
 )
target_link_libraries(skill_base ${catkin_LIBRARIES} )

## Declare a cpp executable
add_executable(skill_manager_node src/skill_manager_node.cpp src/skill_manager.cpp)
target_link_libraries(skill_manager_node skill_base ${catkin_LIBRARIES})
add_dependencies(skill_manager_node skiros_msgs_generate_messages_cpp)
add_dependencies(skill_base skiros_msgs_generate_messages_cpp)

#add_executable(skill_test_node src/skill_test_node.cpp)
#target_link_libraries(skill_test_node skill_base ${catkin_LIBRARIES})
#add_dependencies(skill_test_node skiros_msgs_generate_messages_cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(devices_manager devices_manager_generate_messages_cpp)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 
#install(TARGETS devices_manager
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

# install header files
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#)
