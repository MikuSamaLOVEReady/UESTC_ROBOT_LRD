# `PIBot_ROS`使用说明

> `pibot`是银星合作时购买的小车底盘。其包含直流电机控制、里程计、IMU和上位机通讯等功能。

## 1、远程连接`RK3588`板子

​		我们是通过`RK3588 & RK3566`等高性能的主板控制`PIBot`下位机(`STM32`控制板)。如果直接采用`HDMI`线和`USB`鼠标和键盘进行控制非常的不便; 采用`VNC`等远程桌面的方式对性能的损耗较大，所以采用远程ssh连接一种较好的方式。这里，**我们采用的`vscode`进行ssh远程连接**, 其具有远程图形显示功能以及其他强大的功能。

​		`ssh`连接需要知道目标机器的`ip`地址。理论上如果知道开发板的外网`IP`地址，那么就可以在任意位置连接开发板。由于外网固定`IP`难以获取，所以这里我们**采用局域网ssh连接**。由于`windows`平台的渲染窗口和`linux`不兼容，所以**windows平台的vscode是不支持ssh窗口显示的，需要开启虚拟机或者`linux`主机下面的`vscode`才可以显示窗口**。我们在虚拟机`ubuntu`里面使用`vscode`远程`ssh`连接`RK3588`和虚拟机需要用同个网络，远程连接[教程](http://www.autolabor.com.cn/book/ROSTutorials/di-9-zhang-ji-qi-ren-dao-822a28-shi-4f5329/92-vscodeyuan-cheng-kai-fa.html)：

1. `ip`查询方式终端: 在小车终端里输入 `ifconfig`查看小车的ip地址
2. 将教程里步骤三内容改为：

3. `ssh -X firefly@ip` （-X 为远程显示图形界面需要）
4. 密码为：`firefly`

5. 连接上后选择需要打开的文件夹


## 2、小车驱动编译

#### 2.1 驱动代码位置

​		`PIBot`的ROS支持包的位置在[仓库]([RobotKernal-UESTC](http://logzhan.ticp.io:30000/logzhan/RobotKernal-UESTC))中，其路径为`Code\RK3588\PIbot_ROS`中。我们解压后可以把`PIBot_ROS`移动到`~/`位置，并重新命名为`pibot_ros`（命名可以按照个人习惯），下面的教程以`pibot_ros`为例。

#### 2.2 3588上编译问题（主要是全覆盖部分）

​		`PIBot_ROS/ros_ws/src/ipa_coverage_planning`(全覆盖部分的代码)

```shell
# 进入到ROS的工作空间
cd pibot_ros/ros_ws
# 在pibot_ros/ros_ws路径下执行catkin_make
# ~:/pibot_ros/ros_ws$ catkin_make
catkin_make
```

#### 2.3 根据出现的报错缺少的库问题解决

```shell
# noetic 是ubuntu20.04版本名，其他版本需要更换名字
sudo apt-get install ros-noetic-xxx   
# 例如：sudo apt-get install ros-noetic-opengm
sudo apt-get install ros-noetic-libdlib
sudo apt-get install ros-noetic-cob-navigation
sudo apt-get install coinor-*
```

可参考内容：

> [全覆盖规划算法Coverage Path Planning开源代码ipa_coverage_planning编译-CSDN博客](https://blog.csdn.net/ktigerhero3/article/details/121562049)
>
> [ROS全覆盖规划算法 Coverage Path Planning 采坑-CSDN博客](https://blog.csdn.net/weixin_42179076/article/details/121164350)

## 3、使用说明

#### 3.1 导航的启动

​		`PIBot_ROS`的文件解压或重新命名可以按照个人习惯即可，这里以解压后命名为`pibot_ros`为例，介绍命令启动导航。

```shell
# 进入到pibot_ros的工作空间
cd ~/pibot_ros/ros_ws
# 配置环境变量
source ./devel/setup.bash
# 启动导航文件
# 格式 roslaunch package_name launch_file_name
roslaunch pibot_navigation nav.launch
```

#### 3.2 更改导航地图

​		根据导航需求更改导航地图，地图路径在下图左下角位置`pgm`为地图格式 `yaml`为配置文件:

<img src="./Image/ROS_Using (3).png" alt="Untitled" style="zoom:80%;" />

#### 3.3 小车控制和可视化

​		小车的控制和可视化功能需要**开启两个新的终端，分别在终端输入命令**：

```shell
# 小车的键盘控制,在任意路径执行:
pibot_control
```

```shell
# 导航可视化,在任意路径执行:
pibot_view (需要设置从主机  记不清 后面再补充)
```

​		在这上面可以在地图上给出目标点进行单点导航。

#### 3.4、全覆盖路径规划的使用