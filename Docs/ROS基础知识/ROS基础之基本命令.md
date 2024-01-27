# ROS基础之基础命令

> https://blog.csdn.net/sinat_16643223/article/details/113935412

## 一、常用命令

##### 1.1 `catkin_make`编译命令

```shell
# 进入工作空间
cd ros_ws
# 编译
catkin_make
```

​		我们在使用ROS的时候经常会使用`catkin_make`命令，但我们时常会有疑惑`cakin_make`和`make`的区别。我们可以简单理解为`catkin_make`是`make`的扩展，它更加适合ROS相关的编译。

​		`Catkin` 是 ROS 中使用的构建系统，而 `CMake `是 `Catkin` 的底层构建系统。具体而言，`Catkin` 使用 `CMake` 作为其构建系统的基础，它对 `CMake` 进行了扩展，以适应 ROS 的特殊需求和机器人软件包的组织。

​		以下是它们之间的关系：

1. **`CMake`：** `CMake` 是一个跨平台的开源构建系统，它使用一种基于文本的 DSL（领域特定语言）来描述软件项目的构建过程。`CMake` 生成适用于目标平台的构建系统文件，例如 `Makefile` 或` Visual Studio` 项目文件。
2. **`Catkin`：** Catkin 是 ROS 特有的构建系统，构建在 `CMake` 之上。`Catkin` 扩展了 `CMake`，以便更好地支持 `ROS` 软件包的组织和构建。`Catkin` 引入了一些 ROS 特有的概念，例如工作空间（workspace）、软件包（package）、消息和服务生成等。

​		在一个 `ROS` 工作空间中，通常会包含一个或多个` Catkin` 软件包，每个软件包都包含一个 `CMakeLists.txt` 文件，其中定义了软件包的构建规则、依赖关系等。总的来说，**Catkin 通过扩展 `CMake`，提供了一个更高级别的构建系统，使得在 ROS 中更容易管理和构建机器人软件项目**。 CMake 提供了通用的构建系统功能，而 `Catkin` 在其基础上添加了 `ROS` 特有的工具和概念。

##### 1.2 `roslaunch`命令

```shell
roslaunch pibot_navigation nav.launch
```

​		在命令`roslaunch pibot_navigation nav.launch`中，`roslaunch`本质上是一个可执行文件。`pibot_navigation `是工作空间`workspace`中软件包。`ros_launch`是通过软件包下面的`package.xml`识别出功能包。`nav.launch`是`pibot_navigation`下面`launch`文件夹下的一个`nav.launch`文件，其本质是个一个`xml`文件，描述了要启动的节点以及对应要传递的参数。