cmake_minimum_required(VERSION 3.0.2)
project(ui_node)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gs_msgs
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
  SerialPort
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs gs_msgs
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ui_node
  ${catkin_INCLUDE_DIRS}
)

set(UI_FILES
  ui/mainwindow.ui
)

set(RESOURCE_FILES
  resources/map.qrc
)

set(HEADER_FILES
  include/ui_node/mainwindow.h
  include/ui_node/serialport.h
  include/ui_node/simplepathgenerator.h
  include/ui_node/ros_interface.h
)

set(SRC_FILES
  src/main.cpp
  src/mainwindow.cpp
  src/serialport.cpp
  src/simplepathgenerator.cpp
  src/ros_interface.cpp
)

add_executable(ui_node
  ${SRC_FILES}
  ${HEADER_FILES}
  ${UI_FILES}
  ${RESOURCE_FILES}
)

add_dependencies(ui_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(ui_node
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::SerialPort
)
