cmake_minimum_required(VERSION 3.0.2)
project(common_utils)

set(ROS_DISTRO $ENV{ROS_DISTRO})
if (${ROS_DISTRO} STREQUAL "kinetic")
  add_compile_options(-std=c++11)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(OpenCV)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES common_utils
 #   CATKIN_DEPENDS roscpp
 #   DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRES}
  ${Boost_INCLUDE_DIRES}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
   src/common_utils.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)



