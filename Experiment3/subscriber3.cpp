#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define LINEAR_X 0
void MarrHildreth(const cv::Mat input, cv::Mat output);
void Canny(const cv::Mat input, cv::Mat output);
void HoughLine(const cv::Mat input, cv::Mat output);
void HoughCircle(const cv::Mat input, cv::Mat output);

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{		
	    VideoCapture capture;
        	capture.open(0);//打开zed相机
		
		ROS_WARN("*****START");
        ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
        ros::NodeHandle n;

        //ros::Rate loop_rate(10);//定义速度发布频率
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器

		
        if (!capture.isOpened())
        {
                printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
                return 0;
        }
        waitKey(1000);
        cv::Mat frame;//当前帧图片
        int nFrames = 0;//图片帧数
        int frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
        int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高

	while (ros::ok())
	{
		capture.read(frame);
		if(frame.empty())
		{
			break;
		}
        
		//Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取zed的左目图片
		cv::Mat frIn = frame(cv::Rect(0, 0, frame.cols, frame.rows));//电脑摄像头
		cv::Mat frIn_grey = cv::Mat(frIn.rows,frIn.cols,CV_8U);		
		cv::Mat figure1 = cv::Mat(frIn.rows,frIn.cols,CV_8U);
        cv::Mat figure2 = cv::Mat(frIn.rows,frIn.cols,CV_8U);
        cv::Mat figure3 = cv::Mat(frIn.rows,frIn.cols,CV_8U);
        // 灰度处理
        
		cv::cvtColor(frIn,frIn_grey,CV_BGR2GRAY);	
		// 边缘处理
		MarrHildreth(frIn_grey, figure1);
        // 霍夫线变换
        HoughLine(figure1, figure2);
        
        // 霍夫圆变换
        /*
        Mat src;
        src = cv::imread("./src/image_pkg/src/img1.jpeg",1);
        cv::Mat src_grey = cv::Mat(src.rows,src.cols,CV_8U);		
		cv::Mat src1 = cv::Mat(src.rows,src.cols,CV_8U);
        cv::Mat src2 = cv::Mat(src.rows,src.cols,CV_8U);
        if( !src.data ) printf("No exist image!\n");
        /// Convert it to gray
        cvtColor( src, src_grey, CV_BGR2GRAY );
        MarrHildreth(src_grey, src1);
        HoughCircle(src1, src2);
        /// Reduce the noise so we avoid false circle detection
        GaussianBlur( src_grey, src_grey, Size(9, 9), 2, 2 );

        vector<Vec3f> circles;
        HoughCircles( src_grey, circles, CV_HOUGH_GRADIENT, 1, frIn.rows/8, 200, 100, 0, 0 );
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
        }*/


		imshow("Origin",frIn);
		imshow("Grey",frIn_grey);
		imshow("MH",figure1);
        imshow("HoughLine",figure2);
        //imshow("HoughCircle",src2);
		
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

/////////////////////３.1边缘检测//////////////////
// Marr-Hildreth边缘检测器
// input：原图像
// output：处理后图像
void MarrHildreth(const Mat input, Mat output)
{
    double a = 0, sigma = 1;
    int Laplace[3][3] = {{1,1,1},{1,-8,1},{1,1,1}};
    double Gauss[7][7], figure[input.rows][input.cols];
    int threshold = 128;
    // 高斯算子
    for(int i = 0;i < 7;i++)
    {
        for(int j = 0;j < 7;j++)
        {
            Gauss[i][j] = exp(-((i-3)*(i-3) + (j-3)*(j-3)/(2*sigma*sigma)));
        }
    }
    // 高斯低通滤波
    for(int i = 3;i < input.rows - 3;i++)
    {
        for(int j = 3;j < input.cols - 3;j++)
        {
            for(int k = 0;k < 7;k++)
            {
                for(int l = 0;l < 7;l++)
                {
                    a += Gauss[k][l]*input.at<uchar>(i+k-3,j+l-3);
                }
            }
            figure[i][j] = a;
            a = 0;
        }
    }
    // 拉普拉斯变换　
    for(int i = 1;i < input.rows - 1;i++)
    {
        for(int j = 1;j < input.cols - 1;j++)
        {
            for(int k = 0;k < 3;k++)
            {
                for(int l = 0;l < 3;l++)
                {
                    a += Laplace[k][l]*figure[i+k-1][j+l-1];
                }
            }
            if(a > threshold) output.at<uchar>(i,j) = 255;
            else output.at<uchar>(i,j) = 0;
            a = 0;
        }
    }
}
// canny边缘检测器
// input：原图像
// output：处理后图像
void Canny(const Mat input, Mat output) 
{
    double a = 0, gx = 0, gy=0, sigma = 1;
    double M[input.rows][input.cols], alpha[input.rows][input.cols];// 梯度幅度，方向
    double Gauss[7][7], figure[input.rows][input.cols];
    int sobelx[3][3] = {{-1,-2,-1},{0,0,0},{1,2,1}}, sobely[3][3] = {{-1,0,1},{-2,0,2},{-1,0,1}};
    int threshold = 128;
    // 高斯算子
    for(int i = 0;i < 7;i++)
    {
        for(int j = 0;j < 7;j++)
        {
            Gauss[i][j] = exp(-((i-3)*(i-3) + (j-3)*(j-3)/(2*sigma*sigma)));
        }
    }
    // 高斯低通滤波
    for(int i = 3;i < input.rows - 3;i++)
    {
        for(int j = 3;j < input.cols - 3;j++)
        {
            for(int k = 0;k < 7;k++)
            {
                for(int l = 0;l < 7;l++)
                {
                    a += Gauss[k][l]*input.at<uchar>(i+k-3,j+l-3);
                }
            }
            figure[i][j] = a;
            a = 0;
        }
    }
    for(int i = 1;i < input.rows - 1;i++)
    {
        for(int j = 1;j < input.cols - 1;j++)
        {
            for(int k = 0;k < 3;k++)
            {
                for(int l = 0;l < 3;l++)
                {
                    gx += sobelx[k][l]*input.at<uchar>(i+k-1,j+l-1);
                    gy += sobely[k][l]*input.at<uchar>(i+k-1,j+l-1);
                }
            }
            M[i][j] = sqrt(gx*gx + gy*gy);
            alpha[i][j] = atan(gy/gx);
        }
    }
}
////////////////3.2霍夫变换////////////
// 线变换
// input：原图像(已边缘处理)
// output：处理后图像
void HoughLine(const Mat input, Mat output)
{
    int max_rou = 2*sqrt(input.rows*input.rows + input.cols*input.cols)+1;
    int figure[181][max_rou];
    // 数组初始化
    for(int i = 0;i < 181;i++)
    {
        for(int j = 0;j < max_rou;j++)
        {
            figure[i][j] = 0;
        }
    }
    // 霍夫参数空间
    for(int i = 0;i < input.rows;i++)
    {
        for(int j = 0;j < input.cols;j++)
        {
            output.at<uchar>(i,j) = 0;
            if(input.at<uchar>(i,j) == 255)
            {
                for(int k = 0;k < 181;k++)
                {
                    double rou = i*cos((k-90)*M_PI/180) + j*sin((k-90)*M_PI/180);
                    if((rou - int(rou)) < 0.5) rou = int(rou);
                    else rou = int(rou) + 1;
                    figure[k][int(rou + max_rou/2)]++;
                }
            }
        }
    }
    //
    for(int k = 0;k < 181;k++)
    {
        for(int l = 0;l < max_rou;l++)
        {
            if(figure[k][l] > 120)
            {
                if((k-90) > -45 && (k-90) < 45) 
                {
                    for(int j = 0;j < input.cols;j++)
                    {
                        int x = int((l - max_rou/2 - j*sin((k-90)*M_PI/180))/cos((k-90)*M_PI/180));
                        if (x < input.rows && x >= 0) output.at<uchar>(x,j) = 255;
                    }                    
                }
                else
                {
                    for(int i = 0;i < input.rows;i++)
                    {
                        int y = int((l - max_rou/2 - i*cos((k-90)*M_PI/180))/sin((k-90)*M_PI/180));
                        if(y < input.cols && y >= 0) output.at<uchar>(i,y) = 255;
                    }
                }
            }
        }
    }
}
// 圆变换
// input：原图像(已边缘处理)
// output：处理后图像
void HoughCircle(const Mat input, Mat output)
{
    int max_r = int(sqrt(input.rows*input.rows + input.cols*input.cols));
    printf("%d,%d,%d\n",input.rows,input.cols,max_r);
    //int figure[input.rows][input.cols][max_r];
    int ***figure;// 动态定义三维数组
    figure = new int**[input.rows];
    for(int i = 0;i < input.rows;i++)
    {
        figure[i] = new int*[input.cols];
    }
    for(int i = 0;i < input.rows;i++)
    {
        for(int j = 0;j < input.cols;j++)
        {
            figure[i][j] = new int[max_r];
        }
    }
    // 初始化数组
    for(int x = 0;x < output.rows;x++)
    {
        for(int y = 0;y < output.cols;y++)
        {
            output.at<uchar>(x,y) = 0;
            for(int r = 0;r < max_r;r++)
            {
                figure[x][y][r] = 0;
            }
        }
    }
    // 霍夫参数空间
    for(int x = 0;x < input.rows;x++)
    {
        for(int y = 0;y < input.cols;y++)
        {
            if(input.at<uchar>(x,y) == 255)
            {
                for(int i = 0;i < input.rows;i++)
                {
                    for(int j = 0;j < input.cols;j++)
                    {
                        int r_1 = int(sqrt((i-x)*(i-x) + (j-y)*(j-y))) - 1;
                        figure[i][j][r_1]++;
                    }
                }
            }
        }
    }
   for(int x = 0;x < input.rows;x++)
    {
        for(int y = 0;y < input.cols;y++)
        {
            for(int r = 0;r < max_r;r++)
            {
                if(figure[x][y][r] > 300)
                {
                    for(int theta = 0;theta < 360;theta++)
                    {
                        int x1 = int(x + (r+1)*cos(theta*M_PI/180));
                        int y1 = int(y + (r+1)*sin(theta*M_PI/180));
                        if(x1 >= 0 && y1 >= 0 && x1 < input.rows && y1 < input.cols) output.at<uchar>(x1,y1) = 255;
                    }
                }
            }
        }
    }
    delete []figure;
}
