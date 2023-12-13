# ROS基础笔记

##### 1.1 `ros::Rate`的用法

​		`ros::Rate` 是ROS中用于控制循环频率的工具。在你提到的代码中，`ros::Rate loop_rate(10);` 创建了一个 `ros::Rate` 对象，其目的是控制循环的频率。

```c++
ros::Rate loop_rate(10);  // 设置事件循环的频率为10Hz

while (ros::ok()) {
    // 处理其他任务，这个地方不能处理过于耗时的操作

    ros::spinOnce();    // 处理一次事件循环
    loop_rate.sleep();  // 使节点等待足够的时间以达到设定的频率
}

```

