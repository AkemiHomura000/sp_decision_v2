cmake_minimum_required(VERSION 3.0.2)
project(sp_decision)
# 设置编译选项
# option(DEBUG_MODE "Enable debug mode" OFF)

# # 添加可选的编译宏定义
# if(DEBUG_MODE)
#     add_compile_definitions(DEBUG_MODE)
# endif()


find_package(catkin REQUIRED COMPONENTS
  robot_msg
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sp_decision
#  CATKIN_DEPENDS robot_msg roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  include
  ${CMAKE_CURRENT_SOURCE_DIR} 
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
src/main.cpp
src/perception/blackboard.cpp
src/executor/chassis.cpp
src/decision_node/decision_tree.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

# add_executable(yaml_test
# src/decision_node/yaml_read.cpp
# )
# target_link_libraries(yaml_test 
# ${catkin_LIBRARIES}
# yaml-cpp
# )
