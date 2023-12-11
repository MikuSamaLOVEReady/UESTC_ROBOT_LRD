#!/bin/bash
if [ ! -z "$PIBOT_HOME" ]; then
    PIBOT_HOME_DIR=$PIBOT_HOME
else
    PIBOT_HOME_DIR=~/pibot_ros
fi

echo -e "\033[1;32mpibot home dir:$PIBOT_HOME_DIR"

# http://wiki.ros.org/ROS/Installation/UbuntuMirrors
sudo sh -c 'echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

code_name=$(lsb_release -sc)

if [ "$code_name" = "trusty" ]; then
    ROS_DISTRO="indigo"
elif [ "$code_name" = "xenial" ]; then
    ROS_DISTRO="kinetic"
elif [ "$code_name" = "bionic" ] || [ "$code_name" = "stretch" ]; then
    ROS_DISTRO="melodic"
elif [ "$code_name" = "focal" ] || [ "$code_name" = "buster" ]; then
    ROS_DISTRO="noetic"
else
    echo "PIBOT not support "$code_name
    exit
fi

echo "ros distro:" $ROS_DISTRO

sudo apt-get install -y git cmake unzip vim build-essential udev inetutils-ping iproute2 hostapd
            
sudo apt-get -y --allow-unauthenticated install ros-${ROS_DISTRO}-ros-base ros-${ROS_DISTRO}-slam-gmapping ros-${ROS_DISTRO}-navigation \
            ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-robot-state-publisher \
            ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-teleop-twist-* ros-${ROS_DISTRO}-control-msgs \
            ros-${ROS_DISTRO}-kdl-parser-py ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-hector-mapping \
            ros-${ROS_DISTRO}-robot-pose-ekf ros-${ROS_DISTRO}-slam-karto ros-${ROS_DISTRO}-hector-geotiff ros-${ROS_DISTRO}-hector-trajectory-server \
            ros-${ROS_DISTRO}-usb-cam ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-image-transport-plugins \
            ros-${ROS_DISTRO}-depthimage-to-laserscan ros-${ROS_DISTRO}-openni2* \
            ros-${ROS_DISTRO}-robot-upstart ros-${ROS_DISTRO}-tf-conversions \
            ros-${ROS_DISTRO}-realsense2-camera ros-${ROS_DISTRO}-libuvc* \
            ros-${ROS_DISTRO}-camera-calibration ros-${ROS_DISTRO}-rtabmap* \
            ros-${ROS_DISTRO}-web-video-server ros-${ROS_DISTRO}-roslint ros-${ROS_DISTRO}-laser-filters
            
if [ "$ROS_DISTRO" = "noetic" ]; then
    sudo ln -sf /usr/bin/python3 /usr/bin/python

    # third 
    cd $PIBOT_HOME_DIR/third_party/libuvc && mkdir -p build && cd build && cmake .. && sudo make install

    cd $PIBOT_HOME_DIR
    if [ ! -d $PWD/ros_package ]; then
        mkdir ros_package
    fi
    cd ros_package

    # astra ros package
    if [ -f $PWD/astra.tar.gz ]; then
        tar xzvf $PWD/astra.tar.gz
    else
        git clone https://github.com/orbbec/ros_astra_launch.git
        git clone https://github.com/orbbec/ros_astra_camera.git
    fi

    # frontier_exploration ros package
    # if [ -f $PWD/frontier_exploration.tar.gz ]; then
    #     tar xzvf $PWD/frontier_exploration.tar.gz
    # else
    #     git clone -b $ROS_DISTRO-devel https://github.com/paulbovbel/frontier_exploration.git
    # fi

    # camera_umd
    # if [ -f $PWD/camera_umd.tar.gz ]; then
    #     tar xzvf $PWD/camera_umd.tar.gz
    # else
    #     git clone https://github.com/ros-drivers/camera_umd.git
    # fi

    cd ..
    echo "ln -sf $PWD/ros_package ros_ws/src/ros_package"
    if [ -f ros_ws/src/ros_package ]; then
        rm ros_ws/src/ros_package
    fi

    ln -snf $PWD/ros_package ros_ws/src/ros_package
    sudo apt-get -y --allow-unauthenticated install python3-pip python3-serial
elif [ "$ROS_DISTRO" = "melodic" ]; then
    # third 
    cd $PIBOT_HOME_DIR/third_party/libuvc && mkdir -p build && cd build && cmake .. && sudo make install

    cd $PIBOT_HOME_DIR
    if [ ! -d $PWD/ros_package ]; then
        mkdir ros_package
    fi
    cd ros_package

    # astra ros package
    if [ -f $PWD/astra.tar.gz ]; then
        tar xzvf $PWD/astra.tar.gz
    else
        git clone https://github.com/orbbec/ros_astra_launch.git
        git clone https://github.com/orbbec/ros_astra_camera.git
    fi

    # frontier_exploration ros package
    # if [ -f $PWD/frontier_exploration.tar.gz ]; then
    #     tar xzvf $PWD/frontier_exploration.tar.gz
    # else
    #     git clone -b $ROS_DISTRO-devel https://github.com/paulbovbel/frontier_exploration.git
    # fi
    cd ..

    echo "ln -sf $PWD/ros_package ros_ws/src/ros_package"
    if [ -f ros_ws/src/ros_package ]; then
        rm ros_ws/src/ros_package
    fi
    
    ln -snf $PWD/ros_package ros_ws/src/ros_package

    sudo apt-get -y --allow-unauthenticated install python-pip python-serial \
            ros-${ROS_DISTRO}-freenect-* ros-${ROS_DISTRO}-orocos-kdl ros-${ROS_DISTRO}-camera-umd ros-${ROS_DISTRO}-cartographer-ros
else
    sudo apt-get -y --allow-unauthenticated install python-pip python-serial \ 
            ros-${ROS_DISTRO}-freenect-* ros-${ROS_DISTRO}-orocos-kdl ros-${ROS_DISTRO}-camera-umd ros-${ROS_DISTRO}-cartographer-ros\
            ros-${ROS_DISTRO}-astra-launch ros-${ROS_DISTRO}-astra-camera ros-${ROS_DISTRO}-frontier-exploration
fi

read -s -n1 -p "install ros gui tools?(y/N)" 

if [ "$REPLY" = "y" -o "$REPLY" = "Y" ]; then
    sudo apt-get -y --allow-unauthenticated install ros-${ROS_DISTRO}-rviz ros-${ROS_DISTRO}-rqt-reconfigure ros-${ROS_DISTRO}-rqt-tf-tree \
    ros-${ROS_DISTRO}-image-view

    if [ "$ROS_DISTRO" = "noetic" ]; then
        echo "please run ros_package/make_cartographer.sh to compile cartographer"
    else
        sudo apt-get -y --allow-unauthenticated install ros-${ROS_DISTRO}-cartographer-rviz
    fi
fi
