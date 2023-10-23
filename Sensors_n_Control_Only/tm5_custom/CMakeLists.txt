cmake_minimum_required(VERSION 3.0.2)
project(tm5_custom)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tm_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

install(DIRECTORY config description launch worlds scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
 
