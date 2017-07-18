cmake_minimum_required(VERSION 2.8.3)
project(bender_hand)

find_package(catkin REQUIRED COMPONENTS
  bender_fieldbus
  dynamic_reconfigure
  message_generation
  uchile_msgs
  uchile_srvs
  roscpp
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS uchile_msgs uchile_srvs roscpp rospy std_msgs std_srvs
)


