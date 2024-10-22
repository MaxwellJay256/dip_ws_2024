#include <stdlib.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
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
CameraState state = REALSENSE;

using namespace cv;
using namespace std;

/// @brief 高斯滤波
/// @param input 输入图像
/// @param output 输出图像
/// @param sigma 标准差
void Gaussian(const Mat &input, Mat &output, double sigma)
{
    if (output.rows != input.rows || output.cols != input.cols || output.channels() != input.channels())
        return;
    int kernel_size = 9;
    double gaussian_kernel[kernel_size][kernel_size];

    /*** 第一步：结合实验二，在此处填充高斯滤波代码 ***/
    int center = round(kernel_size / 2);
    double sum = 0.0;

    // 生成高斯模板
    for (int i = 0; i < kernel_size; i++)
    {
        for (int j = 0; j < kernel_size; j++)
        {
            gaussian_kernel[i][j] = exp(-(pow(i - center, 2) + pow(j - center, 2)) / (2 * pow(sigma, 2))) / (2 * CV_PI * pow(sigma, 2));
            sum += gaussian_kernel[i][j];
        }
    }

    // gaussian_kernel 归一化
    for (int i = 0; i < kernel_size; i++)
    {
        for (int j = 0; j < kernel_size; j++)
        {
            gaussian_kernel[i][j] /= sum;
        }
    }

    // output = imput 卷积 gaussian_kernel(直接使用 cv api)
    Mat kernel = Mat(kernel_size, kernel_size, CV_64F, gaussian_kernel);
    filter2D(input, output, -1, kernel);
}

/// @brief 将 BGR 图像转换为 HSV 图像
/// @param input 输入图像路径
/// @param output 输出图像路径
void BGR2HSV(const Mat &input, Mat &output)
{
    if (input.rows != output.rows ||
        input.cols != output.cols ||
        input.channels() != 3 ||
        output.channels() != 3)
        return;

	for(int i = 0; i < input.rows; i++)
	{
		for (int j = 0; j < input.cols; j++)
		{
            /*** 第二步：在此处填充 RGB 转 HSV 代码 ***/
            uchar b = input.at<Vec3b>(i, j)[0];
            uchar g = input.at<Vec3b>(i, j)[1];
            uchar r = input.at<Vec3b>(i, j)[2];

            // 将 b, g, r 转换为相对值
            float b_f = b / 255.0;
            float g_f = g / 255.0;
            float r_f = r / 255.0;
            
            float cmax = max({r_f, g_f, b_f}); // 最大值
            float cmin = min({r_f, g_f, b_f}); // 最小值
            float delta = cmax - cmin; // 最大值与最小值之差

            float h = 0, s = 0;
            float v = cmax;

            if (delta != 0)
            {
                if (cmax == r_f)
                {
                    h = 60 * ((g_f - b_f) / delta);
                }
                else if (cmax == g_f)
                {
                    h = 60 * ((b_f - r_f) / delta + 2);
                }
                else if (cmax == b_f)
                {
                    h = 60 * ((r_f - g_f) / delta + 4);
                }

                if (cmax != 0)
                {
                    s = delta / cmax;
                }
            }

            output.at<Vec3b>(i, j)[0] = static_cast<uchar>(h);
            output.at<Vec3b>(i, j)[1] = static_cast<uchar>(s * 255);
            output.at<Vec3b>(i, j)[2] = static_cast<uchar>(v * 255);
        }
    }
}

/// @brief 手动阈值分割
/// @param hsv_input 输入 HSV 图像
/// @param grey_output 输出灰度图像
/// @param window 用于展示的窗口
void ColorSplitManual(const Mat &hsv_input, Mat &grey_output, const string window)
{
	static int hmin = 0;
	static int hmax = 255;
	static int smin = 0;
	static int smax = 255;
	static int vmin = 0;
	static int vmax = 255;
	createTrackbar("Hmin", window, &hmin, 255);
	createTrackbar("Hmax", window, &hmax, 255);
	createTrackbar("Smin", window, &smin, 255);
	createTrackbar("Smax", window, &smax, 255);
	createTrackbar("Vmin", window, &vmin, 255);
	createTrackbar("Vmax", window, &vmax, 255);

    /*** 第三步：在此处填充阈值分割代码代码 ***/
    for(int i = 0; i < hsv_input.rows; i++)
    {
        for(int j = 0; j < hsv_input.cols; j++)
        {
            uchar h = hsv_input.at<Vec3b>(i, j)[0];
            uchar s = hsv_input.at<Vec3b>(i, j)[1];
            uchar v = hsv_input.at<Vec3b>(i, j)[2];

            if (h >= hmin && h <= hmax && s >= smin && s <= smax && v >= vmin && v <= vmax)
            {
                grey_output.at<uchar>(i, j) = 255;
            }
            else
            {
                grey_output.at<uchar>(i, j) = 0;
            }
        }
    }

    imshow(window, grey_output);
}

/// @brief 自动阈值分割
/// @param hsv_input 输入 HSV 图像
/// @param bgr_output 输出 BGR 图像（绘制了轮廓）
/// @param contours 存放轮廓的 vector
/// @param hmin H 最小值
/// @param hmax H 最大值
/// @param smin S 最小值
/// @param smax S 最大值
/// @param vmin V 最小值
/// @param vmax V 最大值
void ColorSplitAuto(const Mat &hsv_input, Mat &bgr_output, vector<vector<Point>> &contours, int hmin, int hmax, int smin, int smax, int vmin, int vmax)
{
    int rw = hsv_input.rows;
	int cl = hsv_input.cols;
    Mat color_region(rw, cl, CV_8UC1);

    /*** 第五步：利用已知的阈值获取颜色区域二值图 ***/
    for(int i = 0; i < rw; i++)
    {
        for(int j = 0; j < cl; j++)
        {
            uchar h = hsv_input.at<Vec3b>(i, j)[0];
            uchar s = hsv_input.at<Vec3b>(i, j)[1];
            uchar v = hsv_input.at<Vec3b>(i, j)[2];

            if (h >= hmin && h <= hmax && s >= smin && s <= smax && v >= vmin && v <= vmax)
            {
                color_region.at<uchar>(i, j) = 255;
            }
            else
            {
                color_region.at<uchar>(i, j) = 0;
            }
        }
    }

    /* 获取多边形轮廓 */
    vector<Vec4i> hierarchy;
	findContours(color_region, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	vector<vector<Point>> lines(contours.size());
    /* 利用多项式近似平滑轮廓 */
	for (int i = 0; i < contours.size(); i++)
	{
		approxPolyDP(contours[i], lines[i], 9, true);
	}
	drawContours(bgr_output, lines, -1,Scalar(0, 0, 255), 2, 8);
}

/// @brief 获取颜色区域
/// @param input 输入图像路径
/// @param output 输出图像路径
/// @param contour 轮廓
void GetROI(const Mat &input, Mat &output, const vector<vector<Point>> &contour)
{
    /* 第六步：补充获取颜色区域代码，可使用 drawContours 函数 */
    output = input.clone();
    for (int i = 0; i < contour.size(); i++)
    {
        drawContours(output, contour, i, Scalar(0, 0, 255), 2, 8);
    }
}

/// @brief 统计颜色区域像素个数
/// @param input 
/// @return 颜色区域像素个数
int CountROIPixel(const Mat &input)
{
	int cnt = 0;

    /* 第七步：补充获取颜色区域像素个数的代码 */
    for(int i = 0; i < input.rows; i++)
    {
        for(int j = 0; j < input.cols; j++)
        {
            if (input.at<uchar>(i, j) == 255)
            {
                cnt++;
            }
        }
    }

    return cnt;
}

/*** 第四步：在第三步基础上修改各颜色阈值 ***/
// {hmin, hmax, smin, smax, vmin, vmax}
int red_thresh[6] = {250, 255, 43, 255, 46, 255};
int green_thresh[6] = {40, 49, 43, 255, 46, 255};
int blue_thresh[6] = {95, 106, 43, 255, 46, 255};
int yellow_thresh[6] = {17, 21, 43, 255, 46, 255};

Mat frame_msg;
void rcvCameraCallBack(const sensor_msgs::Image::ConstPtr& img)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
    frame_msg = cv_ptr->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exp4"); // 初始化 ROS 节点
    ros::NodeHandle n;
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Subscriber camera_sub;
    VideoCapture capture;
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
        camera_sub = n.subscribe("/camera/color/image_raw",1,rcvCameraCallBack);
    }

    Mat frIn;
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
            frIn = frIn(cv::Rect(0, 0, frIn.cols/2, frIn.rows)); // 截取zed的左目图片
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

        // 空域高斯滤波
        Mat filter(frIn.size(), CV_8UC3);
        Gaussian(frIn, filter, 3);
        // imshow("filter",filter);

        // RGB 转 HSV
        Mat hsv(frIn.size(), CV_8UC3);
        BGR2HSV(filter, hsv);
        // imshow("hsv",hsv);

        // 手动颜色分割
        Mat grey(frIn.rows, frIn.cols, CV_8UC1);
        ColorSplitManual(hsv, grey, "hsv_split");
        // imshow("split", grey);
        
        int colors = 0;
        int maxs_color_num = 0;
        
        Mat tmp_line = frIn.clone();
        Mat tmp_roi = Mat::zeros(frIn.size(), CV_8UC3);

        // 第八步：结合给出的检测红颜色的代码框架，给出控制小车运动的代码
        vector<vector<Point>> contours_r;
        ColorSplitAuto(hsv, tmp_line, contours_r, red_thresh[0], red_thresh[1], red_thresh[2],
                        red_thresh[3], red_thresh[4], red_thresh[5]);
	    GetROI(frIn, tmp_roi, contours_r);
        imshow("split_r", tmp_line);
	    int red_color_num = CountROIPixel(tmp_roi);
        
        vector<vector<Point>> contours_g;
        ColorSplitAuto(hsv, tmp_line, contours_g, green_thresh[0], green_thresh[1], green_thresh[2],
                          green_thresh[3], green_thresh[4], green_thresh[5]);
        GetROI(frIn, tmp_roi, contours_g);
        imshow("split_g", tmp_line);
        int green_color_num = CountROIPixel(tmp_roi);

        vector<vector<Point>> contours_b;
        ColorSplitAuto(hsv, tmp_line, contours_b, blue_thresh[0], blue_thresh[1], blue_thresh[2],
                          blue_thresh[3], blue_thresh[4], blue_thresh[5]);
        GetROI(frIn, tmp_roi, contours_b);
        imshow("split_b", tmp_line);
        int blue_color_num = CountROIPixel(tmp_roi);

        vector<vector<Point>> contours_y;
        ColorSplitAuto(hsv, tmp_line, contours_y, yellow_thresh[0], yellow_thresh[1], yellow_thresh[2],
                          yellow_thresh[3], yellow_thresh[4], yellow_thresh[5]);
        GetROI(frIn, tmp_roi, contours_y);
        imshow("split_y", tmp_line);
        int yellow_color_num = CountROIPixel(tmp_roi);

        printf("red: %d\tgreen: %d\tblue: %d\tyellow: %d\n",
            red_color_num, green_color_num, blue_color_num, yellow_color_num);

        // 红色 - 前进；绿色 - 后退；蓝色 - 左转；黄色 - 右转
        if (red_color_num > green_color_num && red_color_num > blue_color_num && red_color_num > yellow_color_num)
        {
            colors = 0;
            maxs_color_num = red_color_num;
        }
        else if (green_color_num > red_color_num && green_color_num > blue_color_num && green_color_num > yellow_color_num)
        {
            colors = 1;
            maxs_color_num = green_color_num;
        }
        else if (blue_color_num > red_color_num && blue_color_num > green_color_num && blue_color_num > yellow_color_num)
        {
            colors = 2;
            maxs_color_num = blue_color_num;
        }
        else if (yellow_color_num > red_color_num && yellow_color_num > green_color_num && yellow_color_num > blue_color_num)
        {
            colors = 3;
            maxs_color_num = yellow_color_num;
        }

        geometry_msgs::Twist vel;
        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;
        if(maxs_color_num)
        {
            switch(colors)
            {
                case 0:
                    vel.linear.x = 0.2;
                    break;
                case 1:
                    vel.linear.x = -0.2;
                    break;
                case 2:
                    vel.angular.z = 0.2;
                    break;
                case 3:
                    vel.angular.z = -0.2;
                    break;
            }
        }
        vel_pub.publish(vel);

        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}