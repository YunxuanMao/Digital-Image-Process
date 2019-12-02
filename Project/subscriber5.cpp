/////////////数字图像处理课程设计//////////////
// 小车寻找到两个圆柱,并从两个圆柱中间通过
// 运行代码前需启动小车,输入如下命令
// cd ~/dashgo_ws
// roslaunch navigation_imu.launch
// 无小车可以用rviz试图窗口查看机器人,输入如下命令
// rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz
// author@Yunxuan 2019.12.2
////////////////////////////////////////////

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
#define DIFF_center 100  //与中心的最大差值
#define DIFF_square 0.1 //两者面积的最大相对差值
void ROI_GET(cv::Mat src, cv::Mat dst, int square_ROI[2], cv::Point center_ROI[2]);

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    /////////////////静态测试///////////////
    Mat src = imread("./src/image_pkg/src/turnright.jpg");
    Mat dst = Mat(src.rows, src.cols, CV_8UC3);
    int square_ROI[2];
    int center_ROI[2];
    ROI_GET(src, dst, square_ROI, center_ROI);
    int diff = 0;
    for (int i = 0; i < 2; i++)
    {
        diff += center_ROI[i] / 2;
    }
    diff = diff - src.cols / 2; //两个柱子中点与图像中心的距离, 正表示在中心右侧, 负表示在左侧
    printf("%dx%d\n", src.cols, src.rows);
    printf("diff=%d\n", diff);

    ///////////////主函数/////////////
    VideoCapture capture;
    capture.open(0); //打开zed相机

    ROS_WARN("*****START");
    ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
    ros::NodeHandle n;

    ros::Rate loop_rate(10);//定义速度发布频率
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

/////////////////// 目标颜色检测 ////////////////////////
// 检测目标为方形则判断内部颜色
// 输入图像: src, 输出图像: dst
// 感兴趣区域面积: square_ROI[2]
// 感兴趣区域中点坐标: center_ROI[2]
void ROI_GET(cv::Mat src, cv::Mat dst, int square_ROI[2], Point center_ROI[2])
{
    /////////////高斯滤波////////////
    Mat src_blur = Mat(src.rows, src.cols, CV_8UC3);
    GaussianBlur(src, src_blur, cv::Size(5, 5), 0, 0);
    ///////////转换为HSV图像/////////
    Mat src_hsv = Mat(src.rows, src.cols, CV_8UC3);
    cvtColor(src_blur, src_hsv, CV_BGR2HSV);
    //////获得mask区域(红色区域)//////
    Mat mask1 = Mat(src.rows, src.cols, CV_8U);
    Mat mask2 = Mat(src.rows, src.cols, CV_8U);
    Mat mask = Mat(src.rows, src.cols, CV_8U);
    inRange(src_hsv, Scalar(0, 100, 100), Scalar(10, 255, 255), mask1); //mask为二值化图像
    inRange(src_hsv, Scalar(156, 100, 100), Scalar(180, 255, 255), mask2);
    add(mask1, mask2, mask);
    namedWindow("mask", 0);
    resizeWindow("mask", src.cols / 4, src.rows / 4);
    imshow("mask", mask);
    /////////////边缘检测////////////
    vector<vector<Point>> contours; //边缘点
    vector<Vec4i> hierarchy;        //图像拓扑信息
    findContours(mask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    vector<vector<Point>> contours_ROI(2); //感兴趣区域的边界(1是左侧,2是右侧)
    Rect rect[2];                          //感兴趣边界最小包围矩形
    int num_ROI = 0;                       //感兴趣区域边界数量
    for (int i = 0; i < contours.size(); i++)
    {
        Moments mu = moments(contours[i]);      //图像矩
        if (mu.m00 > 100000 && mu.m00 < 500000) //排除小轮廓
        {
            contours_ROI[num_ROI] = contours[i];
            rect[num_ROI] = boundingRect(contours[i]);
            square_ROI[num_ROI] = rect[num_ROI].area();
            printf("%d\n", square_ROI[num_ROI]);
            center_ROI[num_ROI].x = (rect[num_ROI].tl().x + rect[num_ROI].br().x) / 2;
            center_ROI[num_ROI].y = (rect[num_ROI].tl().y + rect[num_ROI].br().y) / 2;
            printf("%d\n", center_ROI[num_ROI]);
            num_ROI++;
        }
    }
    drawContours(src, contours_ROI, -1, CV_RGB(0, 0, 255), 5); //绘制轮廓
    namedWindow("src", 0);
    resizeWindow("src", src.cols / 4, src.rows / 4);
    imshow("src", src);
}
