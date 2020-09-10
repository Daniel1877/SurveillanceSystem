cmake_minimum_required(VERSION 2.8.3)
project(surveillance_system)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  roscpp
  visualization_msgs
  tf
  roslib
  pcl_conversions
  pcl_ros
  sensor_msgs
  roslint
  rqt_gui
  rqt_gui_cpp
  std_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf rqt_gui_cpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(interactive_tf src/interactive_tf)
target_link_libraries(interactive_tf ${catkin_LIBRARIES})

add_executable(recorder src/recorder.cpp)
add_dependencies(recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recorder ${catkin_LIBRARIES})

add_executable(difference src/difference.cpp)
add_dependencies(difference ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(difference ${catkin_LIBRARIES})

add_executable(exec_plug src/exec_plug.cpp)
add_dependencies(exec_plug ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exec_plug ${catkin_LIBRARIES})

add_executable(recorderPlugin src/recorderPlugin.cpp)
add_dependencies(recorderPlugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(recorderPlugin ${catkin_LIBRARIES})

add_executable(differencePlugin src/differencePlugin.cpp)
add_dependencies(differencePlugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(differencePlugin ${catkin_LIBRARIES})

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

roslint_cpp()

set(surveillance_system_SRCS
  src/my_plugin.cpp
)

set(surveillance_system_HDRS
  include/my_plugin.h
)

set(surveillance_system_UIS
  src/my_plugin.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(surveillance_system_MOCS ${surveillance_system_HDRS})
  qt5_wrap_ui(surveillance_system_UIS_H ${surveillance_system_UIS})
else()
  qt4_wrap_cpp(surveillance_system_MOCS ${surveillance_system_HDRS})
  qt4_wrap_ui(surveillance_system_UIS_H ${surveillance_system_UIS})
endif()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    ${surveillance_system_SRCS}
    ${surveillance_system_MOCS}
    ${surveillance_system_UIS_H}
    )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${qt_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()
