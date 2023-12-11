#!/bin/bash
PIBOT_MODEL=
PIBOT_BOARD=
PIBOT_LIDAR=
PIBOT_3DSENSOR=
PIBOT_ADDRESS=

while getopts "m:b:l:c:a:" opt; do
  case $opt in
    m)
        PIBOT_MODEL=$OPTARG
        echo $PIBOT_MODEL;;
    b)
        PIBOT_BOARD=$OPTARG
        echo $PIBOT_BOARD;;
    l)
        PIBOT_LIDAR=$OPTARG
        echo $PIBOT_LIDAR;;
    c)
        PIBOT_3DSENSOR=$OPTARG
        echo $PIBOT_3DSENSOR;;
    a)
        PIBOT_ADDRESS=$OPTARG
        echo $PIBOT_ADDRESS;;
    \?)
        echo "usage: "$0 "-m {MODLE} -b {BOARD_TYPE} -L {LIDAR} -c {CAMERA} -a {ROS_iP}"
        echo -e "\033[1;32m   for example 1: $0 -m apollo -b stm32f1 -l rplidar -c none -a localhost\033[0m"
        echo -e "\033[1;32m   for example 1: $0 -m hades -b stm32f4 -l rplidar -c none -a localhost\033[0m"
        exit 0 ;;
  esac
done

if [ ! -z "$PIBOT_HOME" ]; then
    PIBOT_HOME_DIR=$PIBOT_HOME
else
    PIBOT_HOME_DIR=~/pibot_ros
fi

echo -e "\033[1;32mpibot home dir:$PIBOT_HOME_DIR"

sudo ln -sf $PIBOT_HOME_DIR/pibot_init_env.sh /usr/bin/pibot_init_env
sudo ln -sf $PIBOT_HOME_DIR/pibot_view_env.sh /usr/bin/pibot_view_env
sudo ln -sf $PIBOT_HOME_DIR/pibot_install_ros.sh /usr/bin/pibot_install_ros

if ! [ $PIBOT_ENV_INITIALIZED ]; then
    echo "export PIBOT_ENV_INITIALIZED=1" >> ~/.bashrc
    echo "source ~/.pibotrc" >> ~/.bashrc
fi

#rules
echo -e "\033[1;32msetup pibot modules"
echo " "
sudo rm -rf /etc/udev/rules.d/pibot.rules
sudo rm -rf /etc/udev/rules.d/rplidar.rules
sudo rm -rf /etc/udev/rules.d/ydlidar.rules

sudo cp $PIBOT_HOME_DIR/rules/98-pibot-usb.rules  /etc/udev/rules.d
sudo cp $PIBOT_HOME_DIR/rules/56-orbbec-usb.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""

sudo udevadm control --reload-rules
sudo udevadm trigger
#sudo service udev reload
#sudo service udev restart

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
    echo -e "\033[1;31m PIBOT not support "$code_name"\033[0m"
    exit
fi 


content="#source ros
if [ ! -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then 
    echo \"please run cd $PIBOT_HOME_DIR && ./pibot_install_ros.sh to install ros sdk\"
else
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi
"
echo "${content}" > ~/.pibotrc

#LOCAL_IP=`ifconfig eth0|grep "inet addr:"|awk -F":" '{print $2}'|awk '{print $1}'`
#LOCAL_IP=`ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | awk -F"/" '{print $1}'`

#if [ ! ${LOCAL_IP} ]; then
#    echo "please check network"
#    exit
#fi

LOCAL_IP=`ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | awk -F"/" '{print $1}'`
echo "LOCAL_IP=\`ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print \$2}' | awk -F"/" '{print \$1}'\`" >> ~/.pibotrc

if [ ! ${LOCAL_IP} ]; then
    echo -e "\033[1;31muse 127.0.0.1 as local ip\033[0m"
    LOCAL_IP=127.0.0.1
fi

if [ "$PIBOT_MODEL" == "" ]; then
    echo -e "\033[1;34mplease specify pibot model:\033[1;32m
                            (0: apollo(2wd-diff),
                            1: apolloX(2wd-diff),
                            2: zeus(3wd-omni),
                            3: hera(4wd-diff),
                            4: hades(4wd-mecanum),
                            5: hadesX(4wd-mecanum),
                            other: user defined)\033[1;33m"
    read -p "" PIBOT_INPUT

    PIBOT_MODEL_TYPE="diff"

    if [ "$PIBOT_INPUT" = "0" ]; then
        PIBOT_MODEL='apollo'
    elif [ "$PIBOT_INPUT" = "1" ]; then
        PIBOT_MODEL='apolloX'
    elif [ "$PIBOT_INPUT" = "2" ]; then
        PIBOT_MODEL='zeus'
        PIBOT_MODEL_TYPE="omni"
    elif [ "$PIBOT_INPUT" = "3" ]; then
        PIBOT_MODEL='hera'
    elif [ "$PIBOT_INPUT" = "4" ]; then
        PIBOT_MODEL='hades'
        PIBOT_MODEL_TYPE="omni"
    elif [ "$PIBOT_INPUT" = "5" ]; then
        PIBOT_MODEL='hadesX'
        PIBOT_MODEL_TYPE="omni"
    else
        PIBOT_MODEL=$PIBOT_INPUT 
    fi
fi

if [ $PIBOT_MODEL == 'apollo' ] || [ $PIBOT_MODEL == 'apolloX' ] || [ $PIBOT_MODEL == 'hera' ]; then
    PIBOT_MODEL_TYPE="diff"
else
    PIBOT_MODEL_TYPE="omni"
fi

if [ "$PIBOT_BOARD" == "" ]; then
    echo -e "\033[1;34mplease specify pibot driver board type:\033[1;32m
                            (0: arduino(mega2560),
                            1: stm32f103,
                            2: stm32f407,
                            other: user defined)\033[1;33m"
    read -p "" PIBOT_INPUT

    if [ "$PIBOT_INPUT" = "0" ]; then
        PIBOT_BOARD='arduino'
    elif [ "$PIBOT_INPUT" = "1" ]; then
        PIBOT_BOARD='stm32f1'
    elif [ "$PIBOT_INPUT" = "2" ]; then
        PIBOT_BOARD='stm32f4'
    else
        PIBOT_BOARD=$PIBOT_INPUT
    fi
fi

if [ $PIBOT_BOARD == 'arduino' ] || [ $PIBOT_BOARD == 'stm32f1' ]; then
    PIBOT_DRIVER_BAUDRATE=115200
else
    PIBOT_DRIVER_BAUDRATE=921600
fi

PIBOT_FAKE_LIDAR=0
if [ "$PIBOT_LIDAR" == "" ]; then
    echo -e "\033[1;34mplease specify your pibot lidar:\033[1;32m
                            (0: not config,
                            1: rplidar(a1,a2),
                            2: rplidar(a3),
                            3: eai(x4),
                            4: eai(g4),
                            5: eai(tg15/tg30/tg50),
                            6: xtion,
                            7: astra,
                            8: kinectV1,
                            9: kinectV2,
                            10: rplidar(s1),
                            other: user defined)\033[1;33m"
    read -p "" PIBOT_INPUT

    if [ "$PIBOT_INPUT" = "0" ]; then
        PIBOT_LIDAR='none'
    elif [ "$PIBOT_INPUT" = "1" ]; then
        PIBOT_LIDAR='rplidar'
    elif [ "$PIBOT_INPUT" = "2" ]; then
        PIBOT_LIDAR='rplidar-a3'
    elif [ "$PIBOT_INPUT" = "3" ]; then
        PIBOT_LIDAR='eai-x4'
    elif [ "$PIBOT_INPUT" = "4" ]; then
        PIBOT_LIDAR='eai-g4'
    elif [ "$PIBOT_INPUT" = "5" ]; then
        PIBOT_LIDAR='eai-tgx'
    elif [ "$PIBOT_INPUT" = "6" ]; then
        PIBOT_LIDAR='xtion'
        PIBOT_FAKE_LIDAR=1
    elif [ "$PIBOT_INPUT" = "7" ]; then
        PIBOT_LIDAR='astra'
        PIBOT_FAKE_LIDAR=1
    elif [ "$PIBOT_INPUT" = "8" ]; then
        PIBOT_LIDAR='kinectV1'
        PIBOT_FAKE_LIDAR=1
    elif [ "$PIBOT_INPUT" = "9" ]; then
        PIBOT_LIDAR='kinectV2'
        PIBOT_FAKE_LIDAR=1
    elif [ "$PIBOT_INPUT" = "10" ]; then
        PIBOT_LIDAR='rplidar-s1'
    else
        PIBOT_LIDAR=$PIBOT_INPUT
    fi
fi

if [ "$PIBOT_LIDAR" = "xtion" ]; then
    PIBOT_FAKE_LIDAR=1
elif [ "$PIBOT_LIDAR" = "astra" ]; then
    PIBOT_FAKE_LIDAR=1
elif [ "$PIBOT_LIDAR" = "kinectV1" ]; then
    PIBOT_FAKE_LIDAR=1
elif [ "$PIBOT_LIDAR" = "kinectV2" ]; then
    PIBOT_FAKE_LIDAR=1
    ln -s $PWD/third_party/iai_kinect2 $PWD/ros_ws/src/
else
    if [ -f $PWD/ros_ws/src/iai_kinect2 ]; then
        rm $PWD/ros_ws/src/iai_kinect2
    fi
fi

if [ $PIBOT_FAKE_LIDAR = 1 ]; then
    echo "fake lidar: $PIBOT_LIDAR"
    PIBOT_3DSENSOR='none'
else
    if [ "$PIBOT_3DSENSOR" == "" ]; then
        echo -e "\033[1;34mplease specify your pibot 3dsensor:\033[1;32m
                            (0: not config,
                            1: xtion,
                            2: astra,
                            3: kinectV1,
                            4: kinectV2,
                            5: d435i,
                            other: user defined)\033[1;33m"    
        read -p "" PIBOT_INPUT

        if [ "$PIBOT_INPUT" = "0" ]; then
            PIBOT_3DSENSOR='none'
        elif [ "$PIBOT_INPUT" = "1" ]; then
            PIBOT_3DSENSOR='xtion'
        elif [ "$PIBOT_INPUT" = "2" ]; then
            PIBOT_3DSENSOR='astra'
        elif [ "$PIBOT_INPUT" = "3" ]; then
            PIBOT_3DSENSOR='kinectV1'
        elif [ "$PIBOT_INPUT" = "4" ]; then
            PIBOT_3DSENSOR='kinectV2'
        elif [ "$PIBOT_INPUT" = "5" ]; then
            PIBOT_3DSENSOR='d435i'
        else
            PIBOT_3DSENSOR=$PIBOT_INPUT
        fi
    fi
fi

echo "export PIBOT_HOME=${PIBOT_HOME_DIR}" >> ~/.pibotrc
echo "export ROS_IP=\`echo \$LOCAL_IP\`" >> ~/.pibotrc
echo "export ROS_HOSTNAME=\`echo \$LOCAL_IP\`" >> ~/.pibotrc
echo "export PIBOT_MODEL=${PIBOT_MODEL}" >> ~/.pibotrc
echo "export PIBOT_MODEL_TYPE=${PIBOT_MODEL_TYPE}" >> ~/.pibotrc
echo "export PIBOT_LIDAR=${PIBOT_LIDAR}" >> ~/.pibotrc
echo "export PIBOT_3DSENSOR=${PIBOT_3DSENSOR}" >> ~/.pibotrc
echo "export PIBOT_BOARD=${PIBOT_BOARD}" >> ~/.pibotrc
echo "export PIBOT_DRIVER_BAUDRATE=${PIBOT_DRIVER_BAUDRATE}" >> ~/.pibotrc
echo "export PIBOT_MAPS_DIR=~/maps" >> ~/.pibotrc
echo "export PIBOT_ASTRA_PID=0x\`lsusb | grep "2bc5:05" | awk '{print \$6}' | awk -F":" '{print \$2}'\`" >> ~/.pibotrc

if [ "$PIBOT_ADDRESS" == "" ]; then
    echo -e "\033[1;34mplease specify the current machine(ip:$LOCAL_IP) type:\033[1;32m
                            (0: pibot board,
                            \033[31m1: control PC or Virtual PC\033[1;34m)\033[1;33m" 
    read PIBOT_INPUT
elif [ $PIBOT_ADDRESS == $LOCAL_IP ] || [ $PIBOT_ADDRESS == "localhost" ]; then
    PIBOT_INPUT="0"
else
    PIBOT_INPUT=$PIBOT_ADDRESS
fi

if [ "$PIBOT_INPUT" == "0" ]; then
    ROS_MASTER_IP_STR="\`echo \$LOCAL_IP\`"
    ROS_MASTER_IP=`echo $LOCAL_IP`
else
    if [ "$PIBOT_ADDRESS" != "$LOCAL_IP" ]; then
        echo -e "\033[1;34mplase specify the pibot board ip for commnication(e.g. 192.168.12.1):" 
        read -p "" PIBOT_INPUT
    else
        PIBOT_INPUT=$PIBOT_ADDRESS
    fi
    ROS_MASTER_IP_STR=`echo $PIBOT_INPUT`
    ROS_MASTER_IP=`echo $PIBOT_INPUT`
fi

echo "export ROS_MASTER_URI=`echo http://${ROS_MASTER_IP_STR}:11311`" >> ~/.pibotrc

echo -e "\033[1;35m*****************************************************************"
echo "model:        " $PIBOT_MODEL 
echo "lidar:        " $PIBOT_LIDAR  
echo "local_ip:     " ${LOCAL_IP} 
echo "onboard_ip:   " ${ROS_MASTER_IP}
echo ""
echo -e "please execute \033[1;36;4msource ~/.bashrc\033[1;35m to make the configure effective\033[0m"
echo -e "\033[1;35m*****************************************************************\033[0m"

#echo "source $PIBOT_HOME_DIR/ros_ws/devel/setup.bash" >> ~/.pibotrc 
content="#source pibot
if [ ! -f $PIBOT_HOME_DIR/ros_ws/devel/setup.bash ]; then 
    echo -e \"please run \033[1;36;4mcd $PIBOT_HOME_DIR/ros_ws && catkin_make\033[0m to compile pibot sdk\"
else
    source $PIBOT_HOME_DIR/ros_ws/devel/setup.bash
fi
"
echo "${content}" >> ~/.pibotrc

#alias
echo "alias pibot_bringup='roslaunch pibot_bringup bringup.launch'" >> ~/.pibotrc 
echo "alias pibot_bringup_with_imu='roslaunch pibot_bringup bringup_with_imu.launch'" >> ~/.pibotrc 
echo "alias pibot_lidar='roslaunch pibot_lidar ${PIBOT_LIDAR}.launch'" >> ~/.pibotrc 
echo "alias pibot_base='roslaunch pibot_bringup robot.launch'" >> ~/.pibotrc 
echo "alias pibot_base_with_imu='roslaunch pibot_bringup robot.launch use_imu:=true'" >> ~/.pibotrc 
echo "alias pibot_control='roslaunch pibot_bringup keyboard_teleop.launch'" >> ~/.pibotrc 
echo "alias pibot_joystick='roslaunch pibot_bringup joystick.launch'" >> ~/.pibotrc
echo "alias pibot_holonomic_joystick='roslaunch pibot_bringup joystick.launch holonomic:=true'" >> ~/.pibotrc

echo "alias pibot_configure='rosrun rqt_reconfigure rqt_reconfigure'" >> ~/.pibotrc 
echo "alias pibot_simulator='roslaunch pibot_simulator nav.launch'" >> ~/.pibotrc 

echo "alias pibot_gmapping='roslaunch pibot_navigation gmapping.launch'" >> ~/.pibotrc 
echo "alias pibot_gmapping_with_imu='roslaunch pibot_navigation gmapping.launch use_imu:=true'" >> ~/.pibotrc 
echo "alias pibot_save_map='roslaunch pibot_navigation save_map.launch'" >> ~/.pibotrc 

echo "alias pibot_naviagtion='roslaunch pibot_navigation nav.launch'" >> ~/.pibotrc 
echo "alias pibot_naviagtion_with_imu='roslaunch pibot_navigation nav.launch use_imu:=true'" >> ~/.pibotrc 
echo "alias pibot_view='roslaunch pibot_navigation view_nav.launch'" >> ~/.pibotrc 

echo "alias pibot_cartographer='roslaunch pibot_navigation cartographer.launch'" >> ~/.pibotrc 
echo "alias pibot_cartographer_with_odom='roslaunch pibot_navigation cartographer_with_odom.launch'" >> ~/.pibotrc 
echo "alias pibot_view_cartographer='roslaunch pibot_navigation view_cartographer.launch'" >> ~/.pibotrc 

echo "alias pibot_hector_mapping='roslaunch pibot_navigation hector_mapping.launch'" >> ~/.pibotrc 
echo "alias pibot_hector_mapping_without_imu='roslaunch pibot_navigation hector_mapping_without_odom.launch'" >> ~/.pibotrc 

echo "alias pibot_karto_slam='roslaunch pibot_navigation karto_slam.launch'" >> ~/.pibotrc

echo "alias pibot_3d_mapping='roslaunch pibot_slam_3d rtabmap.launch use_imu:=false localization:=false'" >> ~/.pibotrc
echo "alias pibot_3d_mapping_with_imu='roslaunch pibot_slam_3d rtabmap.launch use_imu:=true localization:=false'" >> ~/.pibotrc
echo "alias pibot_3d_nav='roslaunch pibot_slam_3d rtabmap.launch use_imu:=false localization:=true'" >> ~/.pibotrc
echo "alias pibot_3d_nav_with_imu='roslaunch pibot_slam_3d rtabmap.launch use_imu:=true localization:=true'" >> ~/.pibotrc
echo "alias pibot_3d_view_mapping='roslaunch pibot_slam_3d view.launch localization:=true'" >> ~/.pibotrc
echo "alias pibot_3d_view_nav='roslaunch pibot_slam_3d view.launch localization:=false'" >> ~/.pibotrc
