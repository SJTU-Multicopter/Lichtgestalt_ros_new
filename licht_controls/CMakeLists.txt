cmake_minimum_required(VERSION 2.8.3)
project(licht_controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  licht_classes
  geometry_msgs
  message_generation
  genmsg
  tf
)
add_message_files(
  FILES
  Lichtcommands.msg
  Lichtoutput.msg
  Lichtsetpoints.msg
  Lichtsetpointsraw.msg
  Lichtstate.msg
  Lichtyaw.msg
  data18.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Boost REQUIRED COMPONENTS 
  program_options 
#  bind
  REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES licht_controls
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  message_runtime
  licht_classes
)

include_directories(
  ${catkin_INCLUDE_DIRS}
 include/licht_controls
)


add_executable(Lichtlinker
  src/Lichtlinker.cpp
)
target_link_libraries(Lichtlinker
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(Lichtcontroller
  src/Lichtcontroller.cpp
)
target_link_libraries(Lichtcontroller
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(Lichtestimator
  src/Lichtestimator.cpp
)
target_link_libraries(Lichtestimator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(Lichttune
  src/Lichttune.cpp
)
target_link_libraries(Lichttune
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
