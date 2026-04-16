cmake_minimum_required(VERSION 3.0.2)
project(stm32_bridge_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gs_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs gs_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(stm32_bridge_node
  src/main.cpp
)

add_dependencies(stm32_bridge_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(stm32_bridge_node
  ${catkin_LIBRARIES}
)
