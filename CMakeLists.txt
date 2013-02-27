cmake_minimum_required(VERSION 2.8.3)
project(p2os_teleop)

find_package(catkin REQUIRED roscpp rospy sensor_msgs geometry_msgs)


###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES p2os_teleop
   CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Declare a cpp executable
add_executable(p2os_teleop  src/p2os_teleop.cc)
target_link_libraries(p2os_teleop ${catkin_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS p2os_teleop
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



