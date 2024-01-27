# `PIBot_ROS`说明

> 更新：`zhanli` 2023-12-13

### 一、`PIBot_ROS`简介		

​		`PIBot_ROS`本质上一个封装功能非常完善的功能包，其内部添加了许多已经整合好了功能，更加便于使用。

- `ros_package` : 内置了许多算法的功能包的压缩包
- `pypibot` : `pibot`相关的python支持功能
- `pibot_upstart` : 未知待补充
- `ros_ws` : `ROS`的工作空间 

##### 1.1 `ros_ws`的目录结构

​		下图中`catkin_workspcae`对应即是`ros_ws`文件夹。其下面一般具有`src`、`build`、`devel`等功能。

<img src="http://logzhan.ticp.io:30000/logzhan/PictureHost/raw/branch/main/RobotKernal-UESTC/ROS_File_Structure.png" style="zoom: 50%;" />

​		上述文件的结构和说明如下面的文档所示：

```shell
WorkSpace --- 自定义的工作空间
    |--- build:编译空间，用于存放CMake和catkin的缓存信息、配置信息和其他中间文件。
    |--- devel:开发空间，用于存放编译后生成的目标文件，包括头文件、动态&静态链接库、可执行文件等。
    |--- src: 源码
        |-- package：功能包(ROS基本单元)包含多个节点、库与配置文件，包名所有字母小写，只能由字母、数字与下划线组成
            |-- CMakeLists.txt 配置编译规则，比如源文件、依赖项、目标文件
            |-- package.xml 包信息，比如:包名、版本、作者、依赖项...(以前版本是 manifest.xml)
            |-- scripts 存储python文件
            |-- src 存储C++源文件
            |-- include 头文件
            |-- msg 消息通信格式文件
            |-- srv 服务通信格式文件
            |-- action 动作格式文件
            |-- launch 可一次性运行多个节点 
            |-- config 配置信息
        |-- CMakeLists.txt: 编译的基本配置
```

##### 1.2 `package.xml`		

​		如上图所示，ROS的基本功能都是通过包(`package`)组织的。该文件定义有关软件包的属性，例如软件包名称，版本号，作者，维护者以及对其他catkin软件包的依赖性。请注意，该概念类似于旧版` rosbuild` 构建系统中使用的`manifest.xml`文件。

​		`package.xml`的部分内容如下图所示,  推测ROS系统通过`package.xml`的内容识别对应包的功能命令。

```xml
<?xml version="1.0"?>
<!-- 格式: 以前是 1，推荐使用格式 2 -->
<package format="2">
  <!-- 包名 -->
  <name>demo01_hello_vscode</name>
  <!-- 版本 -->
  <version>0.0.0</version>
  <!-- 描述信息 -->
  ...
</package>
```

