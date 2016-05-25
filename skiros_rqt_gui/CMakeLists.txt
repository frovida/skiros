cmake_minimum_required(VERSION 2.8.3)
project(skiros_rqt_gui)

set(CMAKE_CXX_FLAGS "-std=c++11")
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
rqt_gui
rqt_gui_cpp
skiros_skill
interactive_markers
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(skiros_rqt_gui_SRCS
  src/skiros_gui.cpp
  src/qtree_model.cpp
  src/custom_dialog_window.cpp
)

set(skiros_rqt_gui_HDRS
  include/skiros_rqt_gui/skiros_gui.h
  include/skiros_rqt_gui/qtree_model.h
  include/skiros_rqt_gui/custom_dialog_window.h
)

set(skiros_rqt_gui_UIS
  src/skiros_gui.ui
)

set(skiros_rqt_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(skiros_rqt_gui_MOCS ${skiros_rqt_gui_HDRS})
qt4_wrap_ui(skiros_rqt_gui_UIS_H ${skiros_rqt_gui_UIS})

include_directories(${skiros_rqt_gui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${skiros_rqt_gui_SRCS} ${skiros_rqt_gui_MOCS} ${skiros_rqt_gui_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#install(FILES plugin.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

#install(PROGRAMS scripts/skiros_rqt_gui
#  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
#install(PROGRAMS scripts/skiros_rqt_gui
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
