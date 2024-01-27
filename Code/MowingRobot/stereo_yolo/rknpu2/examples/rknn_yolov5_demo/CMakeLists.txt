cmake_minimum_required(VERSION 3.4.1)

project(rknn_yolov5_demo)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# skip 3rd-party lib dependencies
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--allow-shlib-undefined")

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/rknn_yolov5_demo_${CMAKE_SYSTEM_NAME})

set(CMAKE_SKIP_INSTALL_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

if(CMAKE_C_COMPILER MATCHES "aarch64")
  set(LIB_ARCH aarch64)
else()
  set(LIB_ARCH armhf)
endif()

include_directories(${CMAKE_SOURCE_DIR})

# rknn api
set(RKNN_API_PATH ${CMAKE_SOURCE_DIR}/../../runtime/RK3588/${CMAKE_SYSTEM_NAME}/librknn_api)
set(RKNN_RT_LIB ${RKNN_API_PATH}/${LIB_ARCH}/librknnrt.so)

include_directories(${RKNN_API_PATH}/include)
include_directories(${CMAKE_SOURCE_DIR}/../3rdparty)

# opencv
# set(OpenCV_DIR ${CMAKE_SOURCE_DIR}/../3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)
find_package(OpenCV REQUIRED)

#Threads
find_package(Threads REQUIRED)

# rga
set(RGA_PATH ${CMAKE_SOURCE_DIR}/../3rdparty/rga/RK3588)
set(RGA_LIB ${RGA_PATH}/lib/Linux//${LIB_ARCH}/librga.so)

include_directories(${RGA_PATH}/include)

# mpp
set(MPP_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/mpp)

include_directories(${MPP_PATH}/include)

set(CMAKE_INSTALL_RPATH "lib")

# rknn_yolov5_demo
include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(rknn_yolov5_demo
  src/main.cpp
  src/preprocess.cc
  src/postprocess.cc
  src/detection.cc
  src/ranging.cpp
  # src/video.cpp
)

target_link_libraries(rknn_yolov5_demo
  ${RKNN_RT_LIB}
  ${RGA_LIB}
  ${OpenCV_LIBS}
  Threads::Threads
)

# install target and libraries
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install/rknn_yolov5_demo_${CMAKE_SYSTEM_NAME})
install(TARGETS rknn_yolov5_demo DESTINATION ./)

install(PROGRAMS ${RKNN_RT_LIB} DESTINATION lib)
install(PROGRAMS ${RGA_LIB} DESTINATION lib)
install(DIRECTORY model DESTINATION ./)

