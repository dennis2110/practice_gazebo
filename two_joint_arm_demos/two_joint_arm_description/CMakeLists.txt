cmake_minimum_required(VERSION 2.8.3)
project(two_joint_arm_description)


find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  rviz
)

catkin_package()



#include_directories(
## include
#  ${catkin_INCLUDE_DIRS}
#)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
