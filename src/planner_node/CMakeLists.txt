cmake_minimum_required(VERSION 3.0.2)
project(planner_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  gs_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs gs_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(planner_node src/planner_node.cpp)

add_dependencies(planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)
