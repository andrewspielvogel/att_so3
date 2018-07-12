cmake_minimum_required(VERSION 2.8.3)
project(att_so3)
set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  dscl_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES att_so3
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp std_msgs dscl_msgs
)



## For Eigen library support
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})



add_executable(att_so3_publisher src/att_so3_publisher.cpp src/so3_att.cpp)
target_link_libraries(att_so3_publisher ${catkin_LIBRARIES})
set_target_properties(att_so3_publisher PROPERTIES COMPILE_FLAGS "-O3")
