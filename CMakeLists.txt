cmake_minimum_required(VERSION 2.8.3)
project(skiros2_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  skiros2_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy skiros2_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/skiros2_examples/action_example/test_action_server
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
