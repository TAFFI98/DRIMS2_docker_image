cmake_minimum_required(VERSION 2.8.3)
project(summer_school_demo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  sensor_msgs
  cv_bridge
)


## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs dynamic_reconfigure cv_bridge
)


install(PROGRAMS
  scripts/image_reader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(PROGRAMS
scripts/calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
