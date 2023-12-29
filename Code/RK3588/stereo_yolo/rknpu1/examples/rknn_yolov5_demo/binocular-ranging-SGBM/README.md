# 双目测距

#### 介绍
结合目标检测的双目测距算法，使用SGBM视差计算方法。

#### 软件架构
软件架构说明


#### 安装教程

1.  安装opencv
2.  编译命令  g++ -g -std=c++11 ./*.cpp -o main.out -I /usr/local/include -I /usr/local/include/opencv4 -l opencv_core -l opencv_imgproc -l opencv_ximgproc -l opencv_highgui -l opencv_dnn_objdetect -l opencv_imgcodecs -l opencv_dnn -l opencv_videoio -l opencv_calib3d

#### 使用说明

1.  插上摄像头
2.  摄像头编号为“0”，与main.cpp 中 Ranging r(0)一致。
3.  运行 *.out文件

#### 参与贡献

1.  Fork 本仓库
2.  新建 Feat_xxx 分支
3.  提交代码
4.  新建 Pull Request


#### 特技

1.  使用 Readme\_XXX.md 来支持不同的语言，例如 Readme\_en.md, Readme\_zh.md
2.  Gitee 官方博客 [blog.gitee.com](https://blog.gitee.com)
3.  你可以 [https://gitee.com/explore](https://gitee.com/explore) 这个地址来了解 Gitee 上的优秀开源项目
4.  [GVP](https://gitee.com/gvp) 全称是 Gitee 最有价值开源项目，是综合评定出的优秀开源项目
5.  Gitee 官方提供的使用手册 [https://gitee.com/help](https://gitee.com/help)
6.  Gitee 封面人物是一档用来展示 Gitee 会员风采的栏目 [https://gitee.com/gitee-stars/](https://gitee.com/gitee-stars/)
