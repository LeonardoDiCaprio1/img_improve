cmake_minimum_required(VERSION 3.0.2)
project(image)

find_package(catkin REQUIRED COMPONENTS
  coscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image
#  CATKIN_DEPENDS coscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/image.py
  scripts/monitoring_img.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


