# ROS机器人底盘-pibot_init_env介绍

## 1.多机通讯

因车载主机限制(没有屏幕或者性能有限），无法启动`Rviz`查看建图等信息，[ROS多机的通讯配置](https://www.jianshu.com/p/69815d79d37f)一文讲了具体的原理细节，即配置一个主机和一个从机，把从机的`ROS_MASTER_URI`指向主机即可，一般我把车载的主机配置指定为主机，而用于显示的PC或者虚拟机配置为从机。`PIBOT`提供了一键配置的脚本，按照提示选择即可
[PIBOT使用手册](https://www.jianshu.com/p/acf8c9f69246)可以看到

- 对于车载的主机

  ![img](https://upload-images.jianshu.io/upload_images/3678585-ef46613326ef894a.png?imageMogr2/auto-orient/strip|imageView2/2/w/974/format/webp)

  车载主机

  ```
  machice type
  ```

  为0即为主机

- 对于用于显示的主机或者虚拟机

  ![img](https://upload-images.jianshu.io/upload_images/3678585-48ce45d2ff550a36.png?imageMogr2/auto-orient/strip|imageView2/2/w/941/format/webp)

  控制主机

  ```
  machice type
  ```

  为非0即为从机，同是作为从机，需要新增一个配置主机

  ```
  IP
  ```

## 2.pibot_init_env脚本

​		具体看下pibot_init_env做了什么

- 添加`PIBOT_ENV_INITIALIZED`环境变量

- 添加`source ~/.pibotrc`至`~/.basrc`

  根据`PIBOT_ENV_INITIALIZED`是否定义，保证`source ~/.pibotrc`只被添加一次

  执行`pibot_init_env`后~/.bashrc文件如下（118 119行）

  ![img](https://upload-images.jianshu.io/upload_images/3678585-1ec5d015ad0182af.png?imageMogr2/auto-orient/strip|imageView2/2/w/535/format/webp)

- 添加`udev rules`
  可以在usb插入根据PID/VID生成`/dev/pibot`和`/dev/rplidar`等软连接，而不需指定具体的`/dev/ttyUSBn`

- 根据驱动板型号设置波特率
  最终执行`python ros_ws/src/pibot_bringup/scripts/set_baud.py 115200`或者`python ros_ws/src/pibot_bringup/scripts/set_baud.py 921600`

- 添加`~/.pibotrc`文件

```sh
 source /opt/ros/kinetic/setup.bash
 LOCAL_IP=`ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | awk -F/ '{print $1}'`
 export ROS_IP=`echo $LOCAL_IP`
 export ROS_HOSTNAME=`echo $LOCAL_IP`
 export PIBOT_MODEL=hades
 export PIBOT_LIDAR=rplidar
 export PIBOT_BOARD=stm32f4
 export ROS_MASTER_URI=http://`echo $LOCAL_IP`:11311
 source ~/pibot_ros/ros_ws/devel/setup.bash
```

- `/opt/ros/kinetic/setup.bash` 生效`ROS`的环境变量,如果没安装好`ROS`会报该文件不存在的错误

- 第二行给LOCAL_IP赋值，我们尝试直接在终端输出该变量

  ```
  echo $LOCAL_IP
  ```

  ![img](https://upload-images.jianshu.io/upload_images/3678585-13ce12edde2b2f6d.png?imageMogr2/auto-orient/strip|imageView2/2/w/541/format/webp)

  可以看到得到的值跟 `ifconfig`查看的一致，可以知道该值为当前`IP`

- 第三四行分别`ROS_IP`和`ROS_HOSTNAME`
  这里使用的本地IP

- 第五六七行分别模型名称，雷达名称，驱动板名称

  - 对于模型名称`pibot_bringup/launch/model.launch`会使用到该变量

    ![img](https://upload-images.jianshu.io/upload_images/3678585-cccc6842688fcb4f.png?imageMogr2/auto-orient/strip|imageView2/2/w/792/format/webp)

    从而加载对应的模型文件

  - 对于雷达名称

    ```
    pibot_bringup/launch/robot.launch
    ```

    会使用到该变量

    ![img](https://upload-images.jianshu.io/upload_images/3678585-817b7e9cd1ced57b.png?imageMogr2/auto-orient/strip|imageView2/2/w/667/format/webp)

    从而加载对于的雷达文件

  - 驱动板名称尚未使用（只在运行是设置波特率）

- ```
  export ROS_MASTER_URI=xxxx
  ```

  这里是主机和从机的唯一区别的地方

  - 对于主机可以看到直接使用本地的`IP`,`export ROS_MASTER_URI=http://`echo $LOCAL_IP`:11311`
  - 对于从机直接使用的手动如输入的`IP`,`export ROS_MASTER_URI=http://192.168.2.231:11311`

- 最后一行即为生效`PIBOT`驱动包的环境变量
  **这里需要编译，不然会提示文件不存在**

## 3.总结

​		其实如[ROS多机的通讯配置](https://www.jianshu.com/p/69815d79d37f)所讲就是设置了`ROS_IP` `ROS_HOSTNAME`和`ROS_MASTER_URI`三个环境变量，前2个主机从机都是本机IP，后一个主机为本机IP，从机为主机IP，我们可以输出这几个变量或者使用`pibot_view_env`查看

![img](https://upload-images.jianshu.io/upload_images/3678585-0e3297cfd8afce66.png?imageMogr2/auto-orient/strip|imageView2/2/w/887/format/webp)