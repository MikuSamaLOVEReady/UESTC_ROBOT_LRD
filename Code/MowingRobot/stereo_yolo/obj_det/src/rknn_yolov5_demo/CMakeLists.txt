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
  include/rknn_yolov5_demo/Utils/*
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  # 切换为相对路径
  ../runtime/RK3588/Linux/librknn_api/include
  ../3rdparty/rga/RK3588/include
  #/home/firefly/obj_dec/src/runtime/RK3588/Linux/librknn_api/include
  #/home/firefly/obj_dec/src/3rdparty/rga/RK3588/include
)

add_library(
  head 
  include/rknn_yolov5_demo/detection.h
  include/rknn_yolov5_demo/postprocess.h
  include/rknn_yolov5_demo/preprocess.h
  include/rknn_yolov5_demo/ranging.h
  include/rknn_yolov5_demo/Timer.h
  include/rknn_yolov5_demo/Utils/UESTC_queue.h
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
      #转换为相对路径
      ${CMAKE_SOURCE_DIR}/librknnrt.so
      ${CMAKE_SOURCE_DIR}/librga.so
      #${CMAKE_SOURCE_DIR}/libopencv_highgui.so
)




