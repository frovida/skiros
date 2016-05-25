cmake_minimum_required(VERSION 2.8.3)
project(skiros_world_model)

set(CMAKE_CXX_FLAGS "-std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
)
find_package(catkin REQUIRED COMPONENTS 
  actionlib
  roscpp
  skiros_msgs
  skiros_common
  skiros_config
  pluginlib
  cmake_modules
  tf
)
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
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
  LIBRARIES world_model_interface
  CATKIN_DEPENDS actionlib roscpp skiros_msgs skiros_common skiros_config pluginlib
#  DEPENDS system_lib
)

#catkin_python_setup()

###########
## Build ##
###########

#Redland libraries
set(redland_dir /usr/local/)
include_directories(${redland_dir}/include) # Set Include directories
include_directories(${redland_dir}/include/raptor2) # Set Include directories
include_directories(${redland_dir}/include/rasqal) # Set Include directories
link_directories(${redland_dir}/lib/) # Set Linking directories
link_directories(${redland_dir}/lib/redland) # Set Linking directories
set(EXTRA_LIBS ${EXTRA_LIBS} raptor2 rasqal xml2 rdf) # Set link variable

#pkg_check_modules(REDLAND REQUIRED redland rasqal)# raptor libxml-2.0)
#include_directories(${REDLAND_INCLUDE_DIRS}) # Set Include directories
#set(EXTRA_LIBS ${EXTRA_LIBS} ${REDLAND_LIBRARIES})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

## Declare a cpp library
add_library(world_model_interface
   src/discrete_reasoner.cpp
   src/std_uris.cpp
   src/module_core.cpp
   src/world_element.cpp
   src/world_model_interface.cpp
   src/reasoners_loading_func.cpp
   src/utility.cpp
   src/condition.cpp)

target_link_libraries(world_model_interface ${catkin_LIBRARIES})

add_library(element_param
   src/element_param_plugin.cpp)

target_link_libraries(element_param ${catkin_LIBRARIES})

add_library(owl_world_model
   src/discrete_reasoner.cpp
   src/std_uris.cpp
   src/owl_world_model.cpp
   src/owl_world_ontology.cpp)
target_link_libraries(owl_world_model ${catkin_LIBRARIES} ${EXTRA_LIBS})

## Declare a cpp executable
add_executable(world_model_node src/owl_world_model_node.cpp src/owl_world_model.cpp)
target_link_libraries(world_model_node world_model_interface owl_world_model ${catkin_LIBRARIES})

add_executable(test_ontology src/test_ontology.cpp src/owl_world_model.cpp )
target_link_libraries(test_ontology world_model_interface owl_world_model ${catkin_LIBRARIES})

add_executable(test_wm_interface src/test_world_interface.cpp)
target_link_libraries(test_wm_interface world_model_interface ${catkin_LIBRARIES})

add_executable(uri_header_generator src/uri_header_generator.cpp src/owl_world_model.cpp src/world_element.cpp src/reasoners_loading_func.cpp)
target_link_libraries(uri_header_generator owl_world_model ${catkin_LIBRARIES})

add_executable(populate_wm src/populate_node.cpp)
target_link_libraries(populate_wm world_model_interface  ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(world_model_interface skiros_msgs_generate_messages_cpp)
add_dependencies(world_model_node skiros_msgs_generate_messages_cpp)

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


