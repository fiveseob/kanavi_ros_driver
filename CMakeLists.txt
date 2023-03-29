cmake_minimum_required(VERSION 3.0.2)
project(carnavi)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_conversions
  pcl_ros
  std_msgs
)



## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES carnavi
  CATKIN_DEPENDS roscpp
                 rospy
                 pcl_conversions
                 pcl_ros
                 visualization_msgs
                 std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_R2 src/test_R2.cpp)
target_link_libraries(test_R2 ${catkin_LIBRARIES})
add_executable(testsend src/testsend.cpp)
target_link_libraries(testsend ${catkin_LIBRARIES})
add_executable(test_code src/test_code.cpp)
target_link_libraries(test_code ${catkin_LIBRARIES})
