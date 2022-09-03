cmake_minimum_required(VERSION 2.8.3)
project(arm_ik_wbc_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
  roscpp
)


include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  IkWbcIn.msg
  IkWbcOut.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs 
  roscpp
)




include_directories(include ${catkin_INCLUDE_DIRS})


install(PROGRAMS
  nodes/ik_wbc_arm_demo_node.py
  nodes/interbotix_ik_wbc_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




