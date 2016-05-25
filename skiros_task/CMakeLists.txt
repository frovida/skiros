cmake_minimum_required(VERSION 2.8.3)
project(skiros_task)

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
  skiros_skill
  skiros_world_model
  cmake_modules
)

## System dependencies are found with CMake's conventions (skill-AAU)
find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML REQUIRED)


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
    LIBRARIES task_manager
    CATKIN_DEPENDS actionlib roscpp pluginlib skiros_msgs skiros_common skiros_world_model skiros_skill
   #DEPENDS system_lib
)

#catkin_python_setup()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

## Declare a cpp library
 add_library(task_manager
    src/task_manager.cpp
    src/type_manager.cpp
    src/pddl.cpp
 )
target_link_libraries(task_manager ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})


## Declare a cpp executable
add_executable(task_manager_node src/task_manager_node.cpp)
target_link_libraries(task_manager_node task_manager ${catkin_LIBRARIES})
add_dependencies(task_manager_node skiros_msgs_generate_messages_cpp)


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


