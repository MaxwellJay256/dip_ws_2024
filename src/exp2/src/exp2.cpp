#include <stdlib.h>
#include "exp2.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"
#include <math.h>
#include <cv_bridge/cv_bridge.h>

enum CameraState
{
    COMPUTER = 0,
    ZED,
    REALSENSE
};
CameraState state = COMPUTER;
#define pi 3.1415926
using namespace cv;

void meanFilter(Mat &input)
{
    // 生成模板
    int T_size = 3; // 模板大小, 3 or 9
    Mat Template = Mat::zeros(T_size, T_size, CV_64F); // 初始化模板矩阵
    /*** 第 1 步：在此处填充均值滤波模板 ***/
    for (int i = 0; i < T_size; i++)
    {
        for (int j = 0; j < T_size; j++)
        {
            Template.at<double>(i, j) = 1.0 / (T_size * T_size);
        }
    }

    // 卷积
    Mat output = Mat::zeros(input.size(), CV_8UC1);

    /*** 第 2 步：填充模板与输入图像的卷积代码 ***/
    convolve(input, Template, output);

    output.convertTo(output, CV_8UC1);
    imshow("mean_filtered_image", output);
}

void convolve(cv::Mat &input, cv::Mat &kernel, cv::Mat &output)
{
    int pad = kernel.rows / 2; // 填充大小
    Mat paddedInput;
    copyMakeBorder(input, paddedInput, pad, pad, pad, pad, BORDER_REPLICATE);
    for (int i = pad; i < paddedInput.rows - pad; i++)
    {
        for (int j = pad; j < paddedInput.cols - pad; j++)
        {
            double sum = 0;
            for (int m = -pad; m <= pad; m++)
            {
                for (int n = -pad; n <= pad; n++)
                    sum += paddedInput.at<uchar>(i + n, j + m) * kernel.at<double>(pad + n, pad + m);
            }
            output.at<uchar>(i - pad, j - pad) = sum;
        }
    }
}

/// @brief 空域高斯滤波器函数
void gaussianFilter(Mat &input, double sigma)
{
    //利用高斯函数生成模板
    int T_size = 9;                                    // 模板大小
    Mat Template = Mat::zeros(T_size, T_size, CV_64F); // 初始化模板矩阵
    int center = round(T_size / 2);                    // 模板中心位置
    double sum = 0.0;
    
    for (int i = 0; i < T_size; i++)
    {
        for (int j = 0; j < T_size; j++)
        {
            /*** 第 3 步：在此处填充高斯滤波模板元素计算代码 ***/
            Template.at<double>(i, j) =
                exp(-(pow(i - center, 2) + pow(j - center, 2)) / (2 * pow(sigma, 2))) / (2 * pi * pow(sigma, 2));            
            
            sum += Template.at<double>(i, j); // 用于归一化模板元素
        }
    }

    for (int i = 0; i < T_size; i++)
    {
        for (int j = 0; j < T_size; j++)
        {
            /*** 第 4 步：在此处填充模板归一化代码 ***/
            Template.at<double>(i, j) /= sum;
        }
    }
    // 卷积
    Mat output = Mat::zeros(input.size(), CV_8UC1);

    /*** 第 5 步：同第 2 步，填充模板与输入图像的卷积代码 ***/
    convolve(input, Template, output);

    output.convertTo(output, CV_8UC1);
    imshow("spatial_filtered_image", output);
}

/// @brief 锐化空域滤波
void sharpenFilter(Mat &input)
{
    //生成模板
    int T_size = 3;                                    // 模板大小
    Mat Template = Mat::zeros(T_size, T_size, CV_64F); // 初始化模板矩阵
    /*** 第六步：填充锐化滤波模板 ***/
    // 0 -1 0
    // -1 5 -1
    // 0 -1 0
    Template.at<double>(0, 0) = 0;
    Template.at<double>(0, 1) = -1;
    Template.at<double>(0, 2) = 0;
    Template.at<double>(1, 0) = -1;
    Template.at<double>(1, 1) = 5;
    Template.at<double>(1, 2) = -1;
    Template.at<double>(2, 0) = 0;
    Template.at<double>(2, 1) = -1;
    Template.at<double>(2, 2) = 0;

    // 卷积
    Mat output = Mat::zeros(input.size(), CV_8UC1);

    /*** 第 7 步：同第 2 步，填充模板与输入图像的卷积代码 ***/
    convolve(input, Template, output);

    output.convertTo(output, CV_8UC1);
    imshow("sharpen_filtered_image", output);
}

/// @brief 膨胀函数
void Dilate(Mat &Src)
{
    Mat Dst = Src.clone();
    Dst.convertTo(Dst, CV_8UC1);
    threshold(Dst, Dst, 128, 255, THRESH_BINARY); // 二值化

    /*** 第 8 步：填充膨胀代码 ***/
    int T_size = 3; // 模板大小
    Mat Template = Mat::ones(T_size, T_size, CV_8UC1); // 初始化模板矩阵
    int center = round(T_size / 2);                    // 模板中心位置

    for (int i = center; i < Src.rows - center; i++)
    {
        for (int j = center; j < Src.cols - center; j++)
        {
            int max = 0;
            for (int m = 0; m < T_size; m++)
            {
                for (int n = 0; n < T_size; n++)
                {
                    if (Template.at<uchar>(m, n) == 1)
                    {
                        if (Src.at<uchar>(i + m - center, j + n - center) > max)
                            max = Src.at<uchar>(i + m - center, j + n - center);
                    }
                }
            }
            Dst.at<uchar>(i, j) = max;
        }
    }

    Dst.convertTo(Dst, CV_8UC1);
    imshow("dilate", Dst);
}

/// @brief 腐蚀函数
void Erode(Mat &Src)
{
    Mat Dst = Src.clone();
    Dst.convertTo(Dst, CV_8UC1);
    threshold(Dst, Dst, 128, 255, THRESH_BINARY); // 二值化

    /*** 第 9 步：填充腐蚀代码 ***/
    int T_size = 3; // 模板大小
    Mat Template = Mat::ones(T_size, T_size, CV_8UC1); // 初始化模板矩阵
    int center = round(T_size / 2);                    // 模板中心位置

    for (int i = center; i < Src.rows - center; i++)
    {
        for (int j = center; j < Src.cols - center; j++)
        {
            int min = 255;
            for (int m = 0; m < T_size; m++)
            {
                for (int n = 0; n < T_size; n++)
                {
                    if (Template.at<uchar>(m, n) == 1)
                    {
                        if (Src.at<uchar>(i + m - center, j + n - center) < min)
                            min = Src.at<uchar>(i + m - center, j + n - center);
                    }
                }
            }
            Dst.at<uchar>(i, j) = min;
        }
    }

    Dst.convertTo(Dst, CV_8UC1);
    imshow("erode", Dst);
}

Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr& img)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exp2_node"); // 初始化 ROS 节点
    ros::NodeHandle n;
    ros::Subscriber camera_sub;
    VideoCapture capture;
    capture.open(0);     

    if(state == COMPUTER)
    {
        capture.open(0);     
        if (!capture.isOpened())
        {
            printf("电脑摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == ZED)
    {
        capture.open(4);     
        if (!capture.isOpened())
        {
            printf("ZED摄像头没有正常打开\n");
            return 0;
        }
        waitKey(1000);
    }
    else if(state == REALSENSE)
    {
        camera_sub = n.subscribe("/camera/color/image_raw", 1, rcvCameraCallBack);
    }

    Mat frIn; // 当前帧图片
    while (ros::ok())
    {
        if(state == COMPUTER)
        {
            capture.read(frIn);
            if (frIn.empty())
            {
                printf("没有获取到电脑图像\n");
                continue;
            }
        }
        else if(state == ZED)
        {
            capture.read(frIn);
            if (frIn.empty())
            {
                printf("没有获取到ZED图像\n");
                continue;
            }
            frIn = frIn(cv::Rect(0,0,frIn.cols/2,frIn.rows));//截取zed的左目图片
        }
        else if(state == REALSENSE)
        {
            if(frame_msg.cols == 0)
            {
                printf("没有获取到realsense图像\n");
                ros::spinOnce();
                continue;
            }
            frIn = frame_msg;
        }
        cvtColor(frIn, frIn, COLOR_BGR2GRAY); // 转换为灰度图
        imshow("original_image", frIn);
        
        //空域均值滤波
		meanFilter(frIn);
	
        // 空域高斯滤波
        double sigma = 2.5;
        gaussianFilter(frIn, sigma);

        //空域锐化滤波
        sharpenFilter(frIn);

        // 膨胀函数
        Dilate(frIn);

        // 腐蚀函数
        Erode(frIn);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}
