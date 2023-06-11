cmake_minimum_required(VERSION 3.0.2)
project(image)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/img_cvbridge.py
  scripts/monitoring_img.py
  scripts/img_numpy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

