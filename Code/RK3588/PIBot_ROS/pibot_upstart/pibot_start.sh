#!/bin/bash

log_file=/tmp/pibot-upstart.log
echo "$DATE: pibot-start" >> $log_file
pibotenv=/etc/pibotenv
. $pibotenv

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
    echo "PIBOT not support $code_name" >> $log_file
    exit
fi

echo "SYS_DIST:                $code_name" >> $log_file
echo "ROS_DIST:                $ROS_DISTRO" >> $log_file
echo "LOCAL_IP:                $LOCAL_IP" >> $log_file
echo "ROS_MASTER_URI:          $ROS_MASTER_URI" >> $log_file
echo "ROS_IP:                  $ROS_IP" >> $log_file
echo "HOSTNAME:                $ROS_HOSTNAME" >> $log_file
echo "PIBOT_MODEL:             $PIBOT_MODEL" >> $log_file
echo "PIBOT_LIDAR:             $PIBOT_LIDAR" >> $log_file
echo "PIBOT_BOARD:             $PIBOT_BOARD" >> $log_file

roslaunch pibot_navigation gmapping.launch