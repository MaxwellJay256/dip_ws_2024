# dip_ws

HITSZ 2024 数字图像处理实验

## Environment

- Ubuntu 20.04
- ROS Noetic
- OpenCV
- CMake
- dashgo_ws（存放在实验室电脑）

### Cameras

On robots:
1. ZED 
2. RealSense D435

## Exp1: 熟悉 Linux 和 ROS

1. 学习 Linux 常用命令
2. ROS 创建、编译工程
3. 使用 OpenCV 调用摄像头
4. ROS 消息发布与订阅

都是一些杂乱代码，没有保存。

## Exp2: 图像获取与直方图均衡化

1. 使用 OpenCV 获取摄像头图像（笔记本电脑或机器人）
2. 统计每个灰度下像素个数、绘制直方图
3. 直方图均衡化，对比效果
4. 添加让机器人原地旋转的代码，让机器人动起来

源码路径：[`src/exp1`](./src/exp1/src/)

直方图均衡化方法参考：[HITSZ-NRSL/HITSZ-AutoCourses/digitalImageProcessing - GitHub](https://github.com/HITSZ-NRSL/HITSZ-AutoCourses/tree/master/digitalImageProcessing)

## Exp3: 图像滤波与形态学

1. 空域均值滤波
2. 空域高斯滤波
3. 锐化空域滤波
4. 腐蚀操作
5. 膨胀操作

实验不准调用 OpenCV 提供的 API，例如 `cv::filter2d()`（卷积操作）, `cv::dialate()`（膨胀操作）, `cv::erode()`（腐蚀操作），要求手动实现，烦。

源码路径：[`src/exp2`](./src/exp2/src/)

## Exp4: 图像特征检测

1. 边缘检测（LoG，Sobel，Canny 算法任选，这里手动实现了 Canny 算法）
2. 霍夫线变换，检测给出图片中的直线
3. 霍夫圆变换，检测给出图片中的圆

源码路径：[`src/exp3`](./src/exp3/src/)

实操下来从电脑摄像头读取图像会严重卡顿，所以只读取了图片。
