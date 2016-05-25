cmake_minimum_required(VERSION 2.8.3)
project(skiros_primitive)

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
  skiros_world_model
  nodelet
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/GripperTest.cfg
)

## System dependencies are found with CMake's conventions (skill-AAU)
find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html (universal_robot)
#catkin_python_setup()


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
  LIBRARIES primitive_pack
  CATKIN_DEPENDS actionlib roscpp pluginlib skiros_msgs skiros_common skiros_world_model nodelet
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
 add_library(primitive_pack
   src/primitive_base.cpp
   src/primitive_composer.cpp
 )
target_link_libraries(primitive_pack ${catkin_LIBRARIES})
add_dependencies(primitive_pack skiros_msgs_generate_messages_cpp)

## Declare a cpp library
 add_library(primitive_scheduler
   src/primitive_scheduler_nodelet.cpp
 )
target_link_libraries(primitive_scheduler ${catkin_LIBRARIES})
add_dependencies(primitive_scheduler skiros_msgs_generate_messages_cpp)

## Declare a cpp executable
add_executable(primitive_manager_node src/primitive_manager_node.cpp)
target_link_libraries(primitive_manager_node ${catkin_LIBRARIES})
add_dependencies(primitive_manager_node skiros_msgs_generate_messages_cpp)

add_executable(test_gripper_action src/test_gripper_action.cpp)
target_link_libraries(test_gripper_action ${catkin_LIBRARIES})
add_dependencies(test_gripper_action skiros_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(devices_manager devices_manager_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_driver_node
#   ${catkin_LIBRARIES}
# )

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


