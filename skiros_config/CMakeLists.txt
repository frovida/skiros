cmake_minimum_required(VERSION 2.8.3)
project(skiros_config)

set(CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  tf
  skiros_common
  pluginlib
)

## System dependencies are found with CMake's conventions (skill-AAU)
find_package(
Boost REQUIRED COMPONENTS system
)

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
  LIBRARIES skiros_config
  CATKIN_DEPENDS pluginlib
 # DEPENDS skiros_config
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
add_library(skiros_config
     src/param_types.cpp
)
target_link_libraries(skiros_config ${catkin_LIBRARIES})

add_library(standard_params
     src/standard_params.cpp
)
target_link_libraries(standard_params ${catkin_LIBRARIES})

## Declare a cpp executable
#add_executable(device_mng src/device_mng.cpp)
#target_link_libraries(device_mng ${catkin_LIBRARIES})
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(devices_manager devices_manager_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ur_driver_node
#   ${catkin_LIBRARIES}
# )

