cmake_minimum_required(VERSION 2.8.3)
project(planning_sim_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  intera_core_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  intera_core_msgs
  gazebo_msgs
  cairo_sawyer_interface
)


#############
## Install ##
#############

install(PROGRAMS
   scripts/ik_pick_and_place_demo.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
