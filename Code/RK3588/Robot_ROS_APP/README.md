# `UWB`建图指导

一、代码编译和运行

```shell
# 进入工程目录
cd Robot_ROS_Driver
# 代码的编译
catkin_make
# 环境变量引入
rosrun ros_merge_test ros_merge_test_node
```

​		线程在`main.cpp`中开启：

<img src=".\image\main.png" alt="image-20231212192507303" style="zoom: 67%;" />



​		主要功能是

uwb接收  uwb_trd

建图  map_trd

保存定位数据  align_trd

建图和保存数据需要哪个用哪个

建图：在rosrun之后 出现的图片上按q开始建图，再按一次q停止，可以再同一幅图上重复开始和暂停

保存数据：同样在rosrun之后出现的图片上按w开始保存 再按w停止
