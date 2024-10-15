#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/***************函数声明，相关参数自行修改***************/
/// @brief 边缘检测函数
/// @param input 输入图像
/// @param output 输出图像
void EdgeDetector(Mat &input, Mat &output);
/// @brief 霍夫线变换函数
/// @param input 输入图像
/// @param output 输出图像
void HoughLines(Mat &input, Mat &output);
/// @brief 霍夫圆变换函数
/// @param input 输入图像
/// @param output 输出图像
void HoughCircles(Mat &input, Mat &output);

int main(int argc, char *argv[])
{
    // 读取 data 文件夹下的图片，用于霍夫线变换和霍夫圆变换
    Mat raw_line = imread("./src/exp3/data/lane.png");
    Mat raw_circle = imread("./src/exp3/data/circle.png");
    if (!raw_line.data || !raw_circle.data)
    {
        cout << "error" << endl;
        return 0;
    }
    imshow("raw_line", raw_line);
    imshow("raw_circle", raw_circle);
    Mat gray_line, gray_circle;
    cvtColor(raw_line, gray_line, COLOR_BGR2GRAY);
    cvtColor(raw_circle, gray_circle, COLOR_BGR2GRAY);

    Mat edge_output, line_output, circle_output;
        
    /****************调用边缘检测函数****************/
    EdgeDetector(raw_line, edge_output);
    imshow("Edge Detection", edge_output);

    /***************调用霍夫线变换***************/
    HoughLines(gray_line, line_output);
    imshow("Hough Lines", line_output);

    /***************调用霍夫圆变换***************/
    HoughCircles(gray_circle, circle_output);
    imshow("Hough Circles", circle_output);

    while(waitKey(10) != 27) {} // ESC
    return 0;
}

/***************下面实现 EdgeDetector() 函数***************/
/// @brief 非极大值抑制
/// @param gradX 
/// @param gradY 
/// @param gradXY 
/// @param directImg 
/// @param dst 
void NonMaxSuppression(Mat& gradX, Mat& gradY, Mat& gradXY, Mat& directImg, Mat& dst);

/// @brief 双阈值检测和连接边缘
/// @param low 
/// @param high 
/// @param img 
/// @param dst 
void DoubleThreshold(double low, double high, Mat& img, Mat& dst);

/// @brief 弱边缘点补充连接强边缘点
/// @param img 
void DoubleThresholdLink(Mat& img);

void EdgeDetector(Mat &input, Mat &output)
{
    // 转换为灰度图
    Mat input_gray;
    cvtColor(input, input_gray, COLOR_BGR2GRAY);

    // 高斯平滑滤波
    Mat blurred;
    int ksize = 5; // 高斯核大小
    GaussianBlur(input_gray, blurred, Size(ksize, ksize), 0, 0);

    // 计算图像梯度
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Sobel(blurred, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
    Sobel(blurred, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
    Mat directImg, gradXY;
    divide(grad_y, grad_x, directImg); // 计算梯度方向
    Mat grad_x_abs, grad_y_abs;
    convertScaleAbs(grad_x, grad_x_abs);
    convertScaleAbs(grad_y, grad_y_abs);
    gradXY = grad_x_abs + grad_y_abs;

    // 非极大值抑制
    Mat grad_threshold;
    NonMaxSuppression(grad_x_abs, grad_y_abs, gradXY, directImg, grad_threshold);

    // 双阈值检测和连接边缘
    DoubleThreshold(50, 150, grad_threshold, output);
}

void NonMaxSuppression(Mat& gradX, Mat& gradY, Mat& gradXY, Mat& directImg, Mat& dst)
{
    dst = gradXY.clone();
    for (int i = 1; i < gradX.rows - 1; i++)
    {
        for (int j = 1; j < gradX.cols; j++)
        {
            directImg.at<float>(i, j) = atan(directImg.at<float>(i, j));
            int P[9] = {
                gradXY.at<uchar>(i - 1, j - 1),
                gradXY.at<uchar>(i - 1, j),
                gradXY.at<uchar>(i - 1, j + 1),
                gradXY.at<uchar>(i, j - 1),
                gradXY.at<uchar>(i, j),
                gradXY.at<uchar>(i, j + 1),
                gradXY.at<uchar>(i + 1, j - 1),
                gradXY.at<uchar>(i + 1, j),
                gradXY.at<uchar>(i + 1, j + 1)
            };
            if (directImg.at<float>(i, j) <= CV_PI / 8 && directImg.at<float>(i, j) > -CV_PI / 8) // 边缘法线水平 (-)
            {
                if (P[4] < P[5] || P[4] < P[3])
                    dst.at<uchar>(i, j) = 0;
            }
            else if (directImg.at<float>(i, j) <= 3 * CV_PI / 8 && directImg.at<float>(i, j) > CV_PI / 8) // 边缘法线 45 度 (\)
            {
                if (P[4] < P[2] || P[4] < P[6])
                    dst.at<uchar>(i, j) = 0;
            }
            else if (directImg.at<float>(i, j) <= 5 * CV_PI / 8 && directImg.at<float>(i, j) > 3 * CV_PI / 8) // 边缘法线垂直 (|)
            {
                if (P[4] < P[1] || P[4] < P[7])
                    dst.at<uchar>(i, j) = 0;
            }
            else // 边缘法线 -45 度 (/)
            {
                if (P[4] < P[0] || P[4] < P[8])
                    dst.at<uchar>(i, j) = 0;
            }
        }
    }
}

void DoubleThresholdLink(Mat& img)
{
    for (int j = 1; j < img.rows - 2; j++)
    {
        for (int i = 1; i < img.cols - 2; i++)
        {
            if (img.at<uchar>(j, i) == 255) // 如果是强边缘点
            {
                for (int m = -1; m < 1; m++)
                {
                    for (int n = -1; n < 1; n++)
                    {
                        if (img.at<uchar>(j + m, i + n) != 0 && img.at<uchar>(j + m, i + n) != 255) // 如果是弱边缘点
                            img.at<uchar>(j + m, i + n) = 255;
                    }
                }
                
            }
        }
    }

    for (int j = 0; j < img.rows - 1; j++)
    {
        for (int i = 0; i < img.cols - 1; i++)
        {
            // 如果该点依旧是弱边缘点，且是孤立边缘点，则抑制之
            if (img.at<uchar>(j, i) != 255)
                img.at<uchar>(j, i) = 0;
        }
    }
}

void DoubleThreshold(double low, double high, Mat& img, Mat& dst)
{
    dst = img.clone();

    for (int j = 0; j < img.rows - 1; j++)
    {
        for (int i = 0; i < img.cols - 1; i++)
        {
            double x = double(dst.at<uchar>(j, i));
            if (x > high)
                dst.at<uchar>(j, i) = 255;
            else if (x < low)
                dst.at<uchar>(j, i) = 0;
        }
    }

    DoubleThresholdLink(dst); // 弱边缘点补充连接强边缘点
}

/***************下面实现 HoughLines() 函数***************/

void HoughLines(Mat &input, Mat& output)
{
    Mat edges;
    Canny(input, edges, 50, 150);

    // 霍夫线变换
    int width = edges.cols;
    int height = edges.rows;
    double rho_max = sqrt(width * width + height * height);
    int rho_num = 2 * rho_max;
    int num_theta = 180;

    Mat accumulator = Mat::zeros(rho_num, num_theta, CV_32SC1); // 累加器
    // 在霍夫空间中投票
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (edges.at<uchar>(y, x) > 0)
            {
                for (int theta = 0; theta < num_theta; theta++)
                {
                    double theta_rad = theta * CV_PI / num_theta;
                    double rho = x * cos(theta_rad) + y * sin(theta_rad);
                    int rho_idx = cvRound(rho + rho_max);
                    accumulator.at<int>(rho_idx, theta)++;
                }
            }
        }
    }

    // 选取最大值
    vector<Vec2f> lines;
    int threshold = 150; // 认为是直线的最少投票数
    for (int i = 0; i < rho_num; i++)
    {
        for (int j = 0; j < num_theta; j++)
        {
            if (accumulator.at<int>(i, j) > threshold)
            {
                float rho = i - rho_max;
                float theta = j * CV_PI / num_theta;
                lines.push_back(Vec2f(rho, theta));
            }
        }
    }

    // 在 input 图的基础上绘制直线
    output = input.clone();
    cvtColor(output, output, COLOR_GRAY2BGR);
    for (int i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        line(output, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
    }
}

/***************下面实现 HoughCircles() 函数***************/
void HoughCircles(Mat &input, Mat& output)
{
    Mat edges;
    Canny(input, edges, 50, 150);

    // 霍夫圆变换
    int width = edges.cols;
    int height = edges.rows;
    int maxRadius = min(width, height) / 2;
    int minRadius = 10; // You can adjust this value

    // 累加器
    int numRho = width * height;
    int numTheta = 360;
    Mat accumulator = Mat::zeros(numRho, maxRadius - minRadius, CV_32SC1);

    // 在霍夫空间中投票
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            if (edges.at<uchar>(y, x) > 0)
            {
                for (int r = minRadius; r < maxRadius; r++)
                {
                    for (int theta = 0; theta < numTheta; theta++)
                    {
                        double thetaRad = CV_PI * theta / 180.0;
                        int a = cvRound(x - r * cos(thetaRad));
                        int b = cvRound(y - r * sin(thetaRad));
                        if (a >= 0 && a < width && b >= 0 && b < height)
                        {
                            accumulator.at<int>(b * width + a, r - minRadius)++;
                        }
                    }
                }
            }
        }
    }

    // 选取最大值
    vector<Vec3f> circles;
    int threshold = 200; // You can adjust this threshold
    for (int r = 0; r < maxRadius - minRadius; r++)
    {
        for (int p = 0; p < numRho; p++)
        {
            if (accumulator.at<int>(p, r) > threshold)
            {
                int b = p / width;
                int a = p % width;
                circles.push_back(Vec3f(a, b, r + minRadius));
            }
        }
    }

    // 在 input 图的基础上绘制圆
    output = input.clone();
    cvtColor(output, output, COLOR_GRAY2BGR);
    for (size_t i = 0; i < circles.size(); i++)
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(output, center, radius, Scalar(0, 0, 255), 2, LINE_AA);
    }
}
