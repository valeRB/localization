cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  robot_msgs
#  genmsg
)


## Generate added messages and services
# sgenerate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
 CATKIN_DEPENDS roscpp
    robot_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node ${catkin_LIBRARIES})

