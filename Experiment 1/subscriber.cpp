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
        Mat frame;//当前帧图片
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

		Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取zed的左目图片
		//imshow("1",frIn);	
		// 此处增加直方图均衡化
	
		cv::cvtColor(frIn,frIn,CV_BGR2GRAY);
		imshow("1",frIn);	
		double n[256];//每个灰度像素点个数
		double p[256];//灰度频率
		double s[256];//灰度累计密度
		double s2[256];//重新分配灰度
		int N = frame.rows * frame.cols / 2;//像素点总数
		printf("%d,%d\n",frIn.rows,frIn.cols);
		int i = 0,j = 0,k = 0,gl = 0;
		
		for(i = 0;i  < 256;i++)
		{
			n[i] = 0;
		}

		//统计灰度下像素个数
		for(j = 0;j < frame.rows ;j++)
		{
			for(k = 0;k <  frame.cols / 2;k++)
			{
				gl = frIn.at<uchar>(j,k) ;
				n[gl] ++;
			}
		}

		
		for(i = 0;i < 256;i++)
		{
			p[i] = n[i]/N;//统计灰度频率
			//计算累计密度			
			if(i == 0) s[i] = p[i];
			else s[i] = s[i-1]+p[i];
			//重新计算均值化后灰度
			s2[i] = s[i]*255;
			if(s2[i] - (int)s2[i] < 0.5) s2[i] = (int)s2[i];
			else s2[i] = 1 + (int)s2[i];			
		}


		//更新灰度值
		for(j = 0;j < frame.rows ;j++)
		{
			for(k = 0;k < frame.cols / 2 ;k++)
			{
				gl = frIn.at<uchar>(j,k) ;
				frIn.at<uchar>(j,k) = (int)s2[gl];
			}
		}
  
		 imshow("2",frIn);
		
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


