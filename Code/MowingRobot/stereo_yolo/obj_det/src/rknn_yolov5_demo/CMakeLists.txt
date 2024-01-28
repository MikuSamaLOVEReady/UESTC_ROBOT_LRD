cmake_minimum_required(VERSION 3.0.2)
project(rknn_yolov5_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


find_package(OpenCV  REQUIRED)
find_package(Threads REQUIRED)

add_message_files(
  FILES
  dis_info.msg
  dis_info_array.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(

 CATKIN_DEPENDS roscpp std_msgs message_runtime

)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /home/firefly/obj_dec/src/runtime/RK3588/Linux/librknn_api/include
  /home/firefly/obj_dec/src/3rdparty/rga/RK3588/include
)

add_library(
  head 
  include/rknn_yolov5_demo/detection.h
  include/rknn_yolov5_demo/postprocess.h
  include/rknn_yolov5_demo/preprocess.h
  include/rknn_yolov5_demo/ranging.h
  include/rknn_yolov5_demo/Timer.h
  # include/rknn_yolov5_demo/pub_info.h
  src/detection.cc
  src/postprocess.cc
  src/preprocess.cc
  src/ranging.cc
  # src/pub_info.cc
)

add_dependencies(head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head
  ${catkin_LIBRARIES}
 
)
add_executable(main src/main.cc)
add_executable(sub_dis src/sub_dis.cc)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sub_dis ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(sub_dis 
${catkin_LIBRARIES}
)
target_link_libraries(main
head
${catkin_LIBRARIES}
${OpenCV_LIBS}
Threads::Threads
/home/firefly/obj_dec/src/librknnrt.so
/home/firefly/obj_dec/src/librga.so
/home/firefly/obj_dec/src/libopencv_highgui.so
)




