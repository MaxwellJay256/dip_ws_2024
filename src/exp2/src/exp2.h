#pragma once

/// @brief 自定义卷积运算
/// @param input 输入矩阵
/// @param kernel 卷积核
/// @param output 输出矩阵
void convolve(cv::Mat &input, cv::Mat &kernel, cv::Mat &output);

/// @brief 空域均值滤波函数
/// @param input 输入图像
void meanFilter(Mat &input);

/// @brief 空域高斯滤波器函数
/// @param input 输入图像
/// @param sigma 高斯函数的标准差
void gaussianFilter(Mat &input, double sigma);

/// @brief 锐化空域滤波
/// @param input 输入图像
void sharpenFilter(Mat &input);

/// @brief 膨胀函数
/// @param Src 输入图像
void Dilate(Mat &Src);

/// @brief 腐蚀函数
/// @param Src 输入图像
void Erode(Mat &Src);
