# PIBOT ROS Workspace v2.0

## 安装ROS
```shell
cd ~/pibot_ros/
./pibot_install_ros.sh
source ~/.bashrc
```

## init environment
for master
```shell
cd ~/pibot_ros/
./pibot_init_env.sh # select board lidar 3dsensor
source ~/.bashrc
```

for salve
```shell
cd ~/pibot_ros/
./pibot_init_env.sh # select board lidar 3dsensor
source ~/.bashrc
```

## run example
```
roslaunch pibot_bringup bringup.launch
roslaunch pibot keyboard_teleop.launch
```

then you can control your robot
