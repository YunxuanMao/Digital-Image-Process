#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define LINEAR_X 0
#define N 100
int object_color_control(cv::Mat src, cv::Rect *rec, int *rec_x, int *rec_y, char *rec_color);
double max(double a, double b);
double min(double a, double b);

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    /////////////////静态测试///////////////
    int rec_x[N];
    int rec_y[N];
    char rec_color[N];
    Rect rec[N];
    Mat src = imread("./src/image_pkg/src/img8.jpeg");
    int rec_num = object_color_control(src, rec, rec_x, rec_y, rec_color);
    printf("(%d,%d)\n", src.cols, src.rows);
    for (int i = 0; i < rec_num; i++)
    {
        printf("(%d,%d,%c)(%d,%d)\n", rec_x[i], rec_y[i], rec_color[i],rec[i].width,rec[i].height);
    }

    VideoCapture capture;
    capture.open(0); //打开zed相机

    ROS_WARN("*****START");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
    ros::NodeHandle n;

    //ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义速度发布器

    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
        return 0;
    }
    waitKey(1000);
    cv::Mat frame;                                           //当前帧图片
    int nFrames = 0;                                         //图片帧数
    int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);   //图片宽
    int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //图片高

    while (ros::ok())
    {
        capture.read(frame);
        if (frame.empty())
        {
            break;
        }

        //Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取zed的左目图片
        Mat frIn = frame(cv::Rect(0, 0, frame.cols, frame.rows)); //电脑摄像头
        //imshow("1",frIn);
        //int rec_num = object_color_control(frIn, rec, rec_x, rec_y, rec_color);

            geometry_msgs::Twist cmd_red;

        // 车的速度值设置
        cmd_red.linear.x = LINEAR_X;
        cmd_red.linear.y = 0;
        cmd_red.linear.z = 0;
        cmd_red.angular.x = 0;
        cmd_red.angular.y = 0;
        cmd_red.angular.z = 0.2;

        pub.publish(cmd_red);

        ros::spinOnce();
        //		loop_rate.sleep();
        waitKey(5);
    }

    return 0;
}

// find the maximum
double max(double a, double b)
{
    double c;
    if (a < b)
        c = b;
    else
        c = a;
    return c;
}

// find the minimum
double min(double a, double b)
{
    double c;
    if (a > b)
        c = b;
    else
        c = a;
    return c;
}

/////////////////// 目标颜色检测 ////////////////////////
// 检测目标为方形则判断内部颜色
int object_color_control(cv::Mat src, cv::Rect *rec, int *rec_x, int *rec_y, char *rec_color)
{
    // 高斯滤波
    Mat src_blur = Mat(src.rows, src.cols, CV_8UC3);
    GaussianBlur(src, src_blur, cv::Size(5, 5), 0, 0);
    //imshow("Gauss",src_blur);
    // 二值化处理
    Mat src_grey = Mat(src.rows, src.cols, CV_8U);
    Mat src_edge = Mat(src.rows, src.cols, CV_8U);
    cvtColor(src_blur, src_grey, CV_BGR2GRAY);             //灰度处理
    equalizeHist(src_grey, src_grey);                      //直方图均衡化
    threshold(src_grey, src_edge, 0, 255, CV_THRESH_OTSU); //二值化
    //Canny(src_grey, src_edge, 3, 9, 3);
    //imshow("Canny",src_edge);
    // 边缘检测
    vector<vector<Point>> contours;              //所有边缘点向量
    vector<Vec4i> hierarchy;                     //图像拓扑信息
    vector<vector<Point>> contours_rectangle(N); //所有方形边界点的向量
    findContours(src_edge, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    // 形状判断
    int flag = 0;
    vector<vector<Point>> contours_ploy(contours.size());
    int num_rec = 0; //方形的数量
    for (int i = 0; i < contours.size(); i++)
    {
        Moments mu = moments(contours[i]); //图像矩
        if (mu.m00 > 5000)                 //排除小轮廓
        {
            double pery = arcLength(contours[i], true);                     //多边形周长
            approxPolyDP(contours[i], contours_ploy[i], 0.04 * pery, true); //多边形边缘拟合
            string shape = "unidentified";
            if (contours_ploy[i].size() == 3)
                shape = "triangle";
            else if (contours_ploy[i].size() == 4)
                shape = "rectangle";
            else if (contours_ploy[i].size() == 5)
                shape = "pentagon";
            else
                shape = "circle";
            // 正方形则显示内部颜色信息
            if (shape == "rectangle")
            {
                contours_rectangle[num_rec] = contours[i];

                Mat src_HSV = Mat(src.rows, src.cols, CV_8UC3);
                cvtColor(src_blur, src_HSV, CV_BGR2HSV);                // BGR转化为HSV空间
                int k = 0, w = 0, r = 0, ye = 0, g = 0, b = 0, sum = 0; // 黑，白，红，黄，绿，蓝
                for (int x = 0; x < src.rows; x++)
                {
                    for (int y = 0; y < src.cols; y++)
                    {
                        double distance = pointPolygonTest(contours[i], Point2f(x, y), true); //判断点是否在轮廓内部
                        if (distance >= 0)
                        {
                            int H = src_HSV.at<Vec3b>(x, y)[0];
                            int S = src_HSV.at<Vec3b>(x, y)[1];
                            int V = src_HSV.at<Vec3b>(x, y)[2];
                            if (V >= 0 && V <= 46)
                                k++;
                            else if (S >= 0 && S <= 43 && V >= 46 && V <= 255)
                                w++;
                            else if ((H >= 0 && H <= 10) || (H >= 125 && H <= 180))
                                r++;
                            else if (H >= 11 && H <= 33)
                                ye++;
                            else if (H >= 34 && H <= 90)
                                g++;
                            else if (H >= 91 && H <= 124)
                                b++;
                            //printf("%d,%d,%d\n",H,S,V);
                        }
                    }
                }
                char color = 0;
                double max_color = max(k, max(w, max(r, max(ye, max(g, b)))));
                if (int(max_color) == r)
                    color = 'r';
                else if (int(max_color) == ye)
                    color = 'y';
                else if (int(max_color) == g)
                    color = 'g';
                else if (int(max_color) == b)
                    color = 'b';
                else if (int(max_color) == k)
                    color = 'k';
                else if (int(max_color) == w)
                    color = 'w';
                rec_x[num_rec] = mu.m10 / mu.m00;
                rec_y[num_rec] = mu.m01 / mu.m00;
                rec[num_rec] = boundingRect(contours[i]);
                rec_color[num_rec] = color;
                flag = 1;
                num_rec++;
            }
        }
    }
    if (flag == 1)
    {
        drawContours(src, contours_rectangle, -1, CV_RGB(255, 0, 0), 5); //绘制轮廓
        imshow("src", src);
    }
    if (flag == 0)
    {
        imshow("src", src);
    }
    return num_rec;
}
