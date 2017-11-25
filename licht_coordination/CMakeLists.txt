cmake_minimum_required(VERSION 2.8.3)
project(licht_coordination)

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
#  commands.msg
#  pos_ctrl_sp.msg
#  raw_ctrl_sp.msg
#  trj_ctrl_sp.msg
#  state_est.msg
#  output.msg

)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Boost REQUIRED COMPONENTS 
  program_options 
#  bind
  REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES licht_coordination
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  message_runtime
  licht_classes
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(Lichtcoord_manuel
  src/Lichtcoord_manuel.cpp
)
target_link_libraries(Lichtcoord_manuel
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

