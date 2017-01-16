cmake_minimum_required(VERSION 2.8.3)
project(licht_classes)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES licht_classes
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib ${USB_LIB}
)


include_directories(
#  ${catkin_INCLUDE_DIRS}
 include/licht_classes
)

add_library(licht_classes
  src/Lichtgestalt.cpp
  src/Lichtbedienung.cpp
)
target_link_libraries(licht_classes
  ${USB_LIB}
)

