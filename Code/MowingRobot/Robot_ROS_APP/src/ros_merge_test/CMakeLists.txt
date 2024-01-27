cmake_minimum_required(VERSION 3.0.2)
project(ros_merge_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 查找 Boost 库
find_package(Boost REQUIRED)

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES ros_merge_test
# #  CATKIN_DEPENDS roscpp rospy std_msgs
# #  DEPENDS system_lib
# CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
# )

include_directories(
# include
${OpenCV_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
        include
)

add_library(${PROJECT_NAME} SHARED
        src/uwb.cpp
        src/mapping.cpp
        src/align.cpp
        include/senddata.h src/senddata.cpp)


add_message_files(
    DIRECTORY msg
    FILES
    RawImu.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
add_executable(${PROJECT_NAME}_node src/main.cpp)


target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${Boost_LIBRARIES} pthread ${PROJECT_NAME})


