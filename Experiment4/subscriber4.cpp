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
#define WINDOW_NAME_R "ThresholdR"
#define WINDOW_NAME_G "ThresholdG"
#define WINDOW_NAME_B "ThresholdB"

double max(double a, double b);// 最大值函数
double min(double a, double b);// 最小值函数
void RGB2HSI(cv::Mat RGB);// RGB空间转换特定色度空间
void color_segment(cv::Mat src);// 颜色分割
// 回调函数
void onChangeTrackBarR(int pos, void* usrdata);
void onChangeTrackBarG(int pos, void* usrdata);
void onChangeTrackBarB(int pos, void* usrdata);
void object_color_tracking(cv::Mat src);
char object_color_control(cv::Mat src);

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{		
	//静态功能演示
	cv::Mat src = imread("./src/image_pkg/src/img2.jpeg");
	
	///////////// Color Segmentation ///////////////
	color_segment(src);
	
	///////////// Object Color Tracking //////////////
	cv::Mat src1 = imread("./src/image_pkg/src/img3.jpeg");
	object_color_tracking(src1);

	//动态图像处理
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
		cv::Mat figure = cv::Mat(frIn.rows,frIn.cols,CV_8U);
		cv::cvtColor(frIn,frIn_grey,CV_BGR2GRAY);// 转换为灰度图
		//////////////// 色度空间转换 /////////////////////
		//////////////// RGB to HSV /////////////////////
		RGB2HSI(frIn);
		// 检测指示牌颜色
		char color = object_color_control(frIn);
		
		geometry_msgs::Twist cmd_red;

		// 车的速度值设置
		if(color == 'r')
		{
			printf("Go forward!\n");
			cmd_red.linear.x = 3;
			cmd_red.linear.y = 0;
			cmd_red.linear.z = 0;
			cmd_red.angular.x = 0;
			cmd_red.angular.y = 0;
			cmd_red.angular.z = 0;
		}
		else if(color == 'g')
		{
			printf("Go backward!\n");
			cmd_red.linear.x = 0;
			cmd_red.linear.y = 3;
			cmd_red.linear.z = 0;
			cmd_red.angular.x = 0;
			cmd_red.angular.y = 0;
			cmd_red.angular.z = 0;
		}
		else if(color == 'b')
		{
			printf("Turn right!\n");
			cmd_red.linear.x = 0;
			cmd_red.linear.y = 0;
			cmd_red.linear.z = 0;
			cmd_red.angular.x = 0;
			cmd_red.angular.y = 0;
			cmd_red.angular.z = 5;
		}
		else if(color == 'y')
		{
			printf("Turn left!\n");
			cmd_red.linear.x = 0;
			cmd_red.linear.y = 0;
			cmd_red.linear.z = 0;
			cmd_red.angular.x = 0;
			cmd_red.angular.y = 0;
			cmd_red.angular.z = -5;
		}
		else
		{
			//printf("No object!\n");
			//cmd_red.linear.x = 0;
			//cmd_red.linear.y = 0;
			//cmd_red.linear.z = 0;
			//cmd_red.angular.x = 0;
			//cmd_red.angular.y = 0;
			//cmd_red.angular.z = 0;
		}
		
	
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
	if(a < b) c = b;
	else c = a;
	return c;
}

// find the minimum
double min(double a, double b)
{
	double c;
	if(a > b) c = b;
	else c = a;
	return c;
}

////////////// Transform RGB to HSV /////////////
void RGB2HSI(cv::Mat RGB)
{
	//vector<Mat>RGB, HSV;
	//Mat dst;
	//split(input, RGB);//提取通道信息，但是当未把其他通道设置为0时显示为灰度图，但各通道信息不同
	//////////////显示RGB三通道////////////////
	/*
	vector<Mat>Rchannels, Gchannels, Bchannels;
    //提取通道信息，把orig的通道数据复制到channals
    split(input, Rchannels);
    split(input, Gchannels);
    split(input, Bchannels);
	//imshow("R",m[1]);
	//将其他通道信息设置为0
    Rchannels[1] = 0;
    Rchannels[2] = 0;
    Gchannels[0] = 0;
    Gchannels[2] = 0;
    Bchannels[0] = 0;
    Bchannels[1] = 0;
    //合并通道信息
    merge(Rchannels, RGB[0]);
    merge(Gchannels, RGB[1]);
    merge(Bchannels, RGB[2]);
    //显示各通道信息
    imshow("B",RGB[0]);
    imshow("G",RGB[1]);
    imshow("R",RGB[2]);
	*/
	//////////////Transform RGB to HSV/////////////
	//v=max(r,g,b), s=(v-min(r,g,b))/v
	//
	Mat HSV = cv::Mat(RGB.rows,RGB.cols,CV_8UC3);
	for(int x = 0;x < RGB.rows;x++)
	{
		for(int y = 0;y < RGB.cols;y++)
		{
			double R = double(RGB.at<Vec3b>(x, y)[0])/255;
			double G = double(RGB.at<Vec3b>(x, y)[1])/255;
			double B = double(RGB.at<Vec3b>(x, y)[2])/255;
			double Cmax = max(max(R, B), G);
			double Cmin = min(min(R, B), G);
			double V = Cmax;
			double S = (Cmax - Cmin)/Cmax;
			double H;
			if(Cmax == Cmin) H = 0;
			else if(Cmax == R) H = 60*((G - B)/(Cmax - Cmin) + 0);
			else if(Cmax == G) H = 60*((B - R)/(Cmax - Cmin) + 2);
			else H = 60*((R - G)/(Cmax - Cmin) + 4);
			HSV.at<Vec3b>(x, y)[0] = int(H*255/360);
			HSV.at<Vec3b>(x, y)[1] = int(S*255);
			HSV.at<Vec3b>(x, y)[2] = int(V*255);
		}
	}
	imshow("HSV",HSV);
}
///////////// Color Segmentation //////////////
void color_segment(cv::Mat src)
{
	Mat R = Mat(src.rows, src.cols, CV_8U);
	Mat G = Mat(src.rows, src.cols, CV_8U);
	Mat B = Mat(src.rows, src.cols, CV_8U);
	for(int x = 0;x < src.rows;x++)
	{
		for(int y = 0;y < src.cols;y++)
		{
			R.at<uchar>(x, y) = src.at<Vec3b>(x, y)[0];
			G.at<uchar>(x, y) = src.at<Vec3b>(x, y)[1];
			B.at<uchar>(x, y) = src.at<Vec3b>(x, y)[2];
			//printf("(%d,%d,%d)\n",src.at<Vec3b>(x, y)[0],src.at<Vec3b>(x, y)[1],src.at<Vec3b>(x, y)[2]);
		}
	}
	//trackbar名  
    string trackBarNameR = "posR";
	string trackBarNameG = "posG";
	string trackBarNameB = "posB";
	//trackbar的值  
    static int posTrackBarR = 0;
	static int posTrackBarG = 0;
	static int posTrackBarB = 0;
    //trackbar的最大值  
    int maxValue = 255;
	//新建窗口
	namedWindow(WINDOW_NAME_R,WINDOW_AUTOSIZE);
	namedWindow(WINDOW_NAME_G,WINDOW_AUTOSIZE);
	namedWindow(WINDOW_NAME_B,WINDOW_AUTOSIZE);
	imshow(WINDOW_NAME_R, R);
	imshow(WINDOW_NAME_G, G);
	imshow(WINDOW_NAME_B, B);
	//创建trackbar，我们把R,G,B作为数据传进回调函数中  
    createTrackbar(trackBarNameR, WINDOW_NAME_R, &posTrackBarR, maxValue, onChangeTrackBarR, &R);
    createTrackbar(trackBarNameG, WINDOW_NAME_G, &posTrackBarG, maxValue, onChangeTrackBarG, &G);
	createTrackbar(trackBarNameB, WINDOW_NAME_B, &posTrackBarB, maxValue, onChangeTrackBarB, &B);
	waitKey();
}
// 回调函数  
void onChangeTrackBarR(int pos, void* usrdata)
{
    Mat src = *(Mat*)(usrdata);
	Mat dst = Mat(src.rows,src.cols,CV_8U);
    // 二值化  
    for(int x = 0;x < src.rows;x++)
	{
		for(int y = 0;y < src.cols;y++)
		{
			if(src.at<uchar>(x, y) >= pos) dst.at<uchar>(x, y) = 255;
			else dst.at<uchar>(x, y) = 0;
		}
	}
	
	if(dst.rows > 0 && dst.cols > 0) imshow(WINDOW_NAME_R, dst);
}
void onChangeTrackBarG(int pos, void* usrdata)
{
    Mat src = *(Mat*)(usrdata);
	Mat dst = Mat(src.rows,src.cols,CV_8U);
    // 二值化  
    for(int x = 0;x < src.rows;x++)
	{
		for(int y = 0;y < src.cols;y++)
		{
			if(src.at<uchar>(x, y) >= pos) dst.at<uchar>(x, y) = 255;
			else dst.at<uchar>(x, y) = 0;
		}
	}
    if(dst.rows > 0 && dst.cols > 0) imshow(WINDOW_NAME_G, dst);
}
void onChangeTrackBarB(int pos, void* usrdata)
{
    Mat src = *(Mat*)(usrdata);
	Mat dst = Mat(src.rows,src.cols,CV_8U);
    // 二值化  
    for(int x = 0;x < src.rows;x++)
	{
		for(int y = 0;y < src.cols;y++)
		{
			if(src.at<uchar>(x, y) >= pos) dst.at<uchar>(x, y) = 255;
			else dst.at<uchar>(x, y) = 0;
		}
	}
	if(dst.rows > 0 && dst.cols > 0) imshow(WINDOW_NAME_B, dst);
}

/////////////////// 目标颜色检测 ////////////////////////
// 检测目标为方形则判断内部颜色
void object_color_tracking(cv::Mat src)
{
	// 高斯滤波
	Mat src_blur = Mat(src.rows, src.cols, CV_8UC3);
	GaussianBlur(src, src_blur, cv::Size(5,5),0,0);
	imshow("Gauss",src_blur);
	// 二值化处理
	Mat src_grey = Mat(src.rows, src.cols, CV_8U);
	Mat src_edge = Mat(src.rows, src.cols, CV_8U);
   	cvtColor(src_blur,src_grey,CV_BGR2GRAY);//灰度处理
	threshold(src_grey,src_edge,200,255,THRESH_BINARY);//二值化
	//Canny(src_grey, src_edge, 3, 9, 3);
	imshow("Canny",src_edge);
	// 边缘检测
	vector<vector<Point>> contours;//所有边缘点向量
	vector<Vec4i> hierarchy;//图像拓扑信息
	findContours(src_edge,contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	// 形状判断
	vector<vector<Point>> contours_ploy(contours.size());
	for(int i = 0;i < contours.size();i++)
	{
		Moments mu = moments(contours[i]);//图像矩
		if(mu.m00 < 50000)//排除小轮廓
		{
			double pery = arcLength(contours[i],true);//多边形周长
			approxPolyDP(contours[i], contours_ploy[i], 0.04*pery, true);//多边形边缘拟合
			string shape = "unidentified";
			if(contours_ploy[i].size() == 3) shape = "triangle";
			else if(contours_ploy[i].size() == 4) shape = "rectangle";
			else if(contours_ploy[i].size() == 5) shape = "pentagon";
			else shape = "circle";
			// 正方形则显示内部颜色信息
			if(shape == "rectangle")
			{
				Mat src_HSV = Mat(src.rows, src.cols, CV_8UC3);
				cvtColor(src_blur,src_HSV,CV_BGR2HSV);// RGB转化为HSV空间
				int k = 0, w = 0, r = 0, ye = 0, g = 0, b = 0, sum = 0;// 黑，白，红，黄，绿，蓝
				for(int x = 0;x < src.rows;x++)
				{
					for(int y = 0 ;y < src.cols;y++)
					{
						double distance = pointPolygonTest(contours[i], Point2f(x, y), true);//判断点是否在轮廓内部
						if(distance >= 0)
						{
							int H = src_HSV.at<Vec3b>(x,y)[0];
							int S = src_HSV.at<Vec3b>(x,y)[1];
							int V = src_HSV.at<Vec3b>(x,y)[2];	
							if(V>=0 && V<=46) k++;
							else if(S>=0 && S<=43 && V>=46 && V<=255) w++;
							else if((H>=0 && H<=10)||(H>=125 && H<=180)) r++;
							else if(H>=11 && H<=33) ye++;
							else if(H>=34 && H<=90) g++;
							else if(H>=91 && H<=124) b++;
							//printf("%d,%d,%d\n",H,S,V);
							sum++;
						}
					}
				}
				printf("%d,%d,%d,%d,%d,%d,%d\n",k,w,r,ye,g,b,sum);
				k = k*src.rows/sum;
				w = w*src.rows/sum;
				r = r*src.rows/sum;
				ye = ye*src.rows/sum;
				g = g*src.rows/sum;
				b = b*src.rows/sum;
				printf("%d,%d,%d,%d,%d,%d\n",k,w,r,ye,g,b);
				//绘制颜色分布图
				Mat Color_segmentation = Mat(src.rows, 500, CV_8UC3);
				for(int x = 0;x < 100;x++)//白色
				{
					for(int y = 0;y < src.rows;y++)
					{
						if(y >= src.rows - w)
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 255;
							Color_segmentation.at<Vec3b>(y,x)[1] = 255;
							Color_segmentation.at<Vec3b>(y,x)[2] = 255;
						}
						else
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 0;
							Color_segmentation.at<Vec3b>(y,x)[1] = 0;
							Color_segmentation.at<Vec3b>(y,x)[2] = 0;
						}
					}
				}
				for(int x = 100;x < 200;x++)//红色
				{
					for(int y = 0;y < src.rows;y++)
					{
						if(y >= src.rows - r)
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 0;
							Color_segmentation.at<Vec3b>(y,x)[1] = 0;
							Color_segmentation.at<Vec3b>(y,x)[2] = 255;
						}
						else
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 0;
							Color_segmentation.at<Vec3b>(y,x)[1] = 0;
							Color_segmentation.at<Vec3b>(y,x)[2] = 0;
						}
					}
				}
				for(int x = 200;x < 300;x++)//黄色
				{
					for(int y = 0;y < src.rows;y++)
					{
						if(y >= src.rows - ye)
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 0;
							Color_segmentation.at<Vec3b>(y,x)[1] = 255;
							Color_segmentation.at<Vec3b>(y,x)[2] = 255;
						}
						else
						{
							Color_segmentation.at<Vec3b>(x,y)[0] = 0;
							Color_segmentation.at<Vec3b>(x,y)[1] = 0;
							Color_segmentation.at<Vec3b>(x,y)[2] = 0;
						}
					}
				}
				for(int x = 300;x < 400;x++)//绿色
				{
					for(int y = 0;y < src.rows;y++)
					{
						if(y >= src.rows - g)
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 0;
							Color_segmentation.at<Vec3b>(y,x)[1] = 255;
							Color_segmentation.at<Vec3b>(y,x)[2] = 0;
						}
						else
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 0;
							Color_segmentation.at<Vec3b>(y,x)[1] = 0;
							Color_segmentation.at<Vec3b>(y,x)[2] = 0;
						}
					}
				}
				for(int x = 400;x < 500;x++)//蓝色
				{
					for(int y = 0;y < src.rows;y++)
					{
						if(y >= src.rows - b)
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 255;
							Color_segmentation.at<Vec3b>(y,x)[1] = 0;
							Color_segmentation.at<Vec3b>(y,x)[2] = 0;
						}
						else
						{
							Color_segmentation.at<Vec3b>(y,x)[0] = 0;
							Color_segmentation.at<Vec3b>(y,x)[1] = 0;
							Color_segmentation.at<Vec3b>(y,x)[2] = 0;
						}
					}
				}
				vector<vector<Point>> contours_rec(1);
				contours_rec[0] = contours[i];
				drawContours(src, contours_rec , -1, CV_RGB(255, 0, 0),5);//绘制轮廓
				imshow("Org",src);
				imshow("Color",Color_segmentation);
				break;
			}
		}
	}
}
/////////////////// 目标颜色检测 ////////////////////////
// 检测目标为方形则判断内部颜色
char object_color_control(cv::Mat src)
{
	// 高斯滤波
	Mat src_blur = Mat(src.rows, src.cols, CV_8UC3);
	GaussianBlur(src, src_blur, cv::Size(5,5),0,0);
	imshow("Gauss",src_blur);
	// 二值化处理
	Mat src_grey = Mat(src.rows, src.cols, CV_8U);
	Mat src_edge = Mat(src.rows, src.cols, CV_8U);
   	cvtColor(src_blur,src_grey,CV_BGR2GRAY);//灰度处理
	threshold(src_grey,src_edge,200,255,THRESH_BINARY);//二值化
	//Canny(src_grey, src_edge, 3, 9, 3);
	imshow("Canny",src_edge);
	// 边缘检测
	vector<vector<Point>> contours;//所有边缘点向量
	vector<Vec4i> hierarchy;//图像拓扑信息
	findContours(src_edge,contours,hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	// 形状判断
	int flag =0;
	vector<vector<Point>> contours_ploy(contours.size());
	for(int i = 0;i < contours.size();i++)
	{
		Moments mu = moments(contours[i]);//图像矩
		if(mu.m00 > 5000)//排除小轮廓
		{
			double pery = arcLength(contours[i],true);//多边形周长
			approxPolyDP(contours[i], contours_ploy[i], 0.04*pery, true);//多边形边缘拟合
			string shape = "unidentified";
			if(contours_ploy[i].size() == 3) shape = "triangle";
			else if(contours_ploy[i].size() == 4) shape = "rectangle";
			else if(contours_ploy[i].size() == 5) shape = "pentagon";
			else shape = "circle";
			// 正方形则显示内部颜色信息
			if(shape == "rectangle")
			{
				Mat src_HSV = Mat(src.rows, src.cols, CV_8UC3);
				cvtColor(src_blur,src_HSV,CV_BGR2HSV);// RGB转化为HSV空间
				int k = 0, w = 0, r = 0, ye = 0, g = 0, b = 0, sum = 0;// 黑，白，红，黄，绿，蓝
				for(int x = 0;x < src.rows;x++)
				{
					for(int y = 0 ;y < src.cols;y++)
					{
						double distance = pointPolygonTest(contours[i], Point2f(x, y), true);//判断点是否在轮廓内部
						if(distance >= 0)
						{
							int H = src_HSV.at<Vec3b>(x,y)[0];
							int S = src_HSV.at<Vec3b>(x,y)[1];
							int V = src_HSV.at<Vec3b>(x,y)[2];	
							if(V>=0 && V<=46) k++;
							else if(S>=0 && S<=43 && V>=46 && V<=255) w++;
							else if((H>=0 && H<=10)||(H>=125 && H<=180)) r++;
							else if(H>=11 && H<=33) ye++;
							else if(H>=34 && H<=90) g++;
							else if(H>=91 && H<=124) b++;
							//printf("%d,%d,%d\n",H,S,V);
						}
					}
				}
				char color = 0;
				double max_color = max(w,max(r,max(ye,max(g,b))));
				if(int(max_color) == r) color = 'r';
				else if(int(max_color) == ye) color = 'y';
				else if(int(max_color) == g) color = 'g';
				else if(int(max_color) == b) color = 'b';
				printf("%d,%d,%d,%d,%d,%d\n%d,%c\n",k,w,r,ye,g,b,int(max_color),color);
				flag = 1;
				vector<vector<Point>> contours_rec(1);
				contours_rec[0] = contours[i];
				drawContours(src, contours_rec , -1, CV_RGB(255, 0, 0),5);//绘制轮廓
				imshow("src",src);
				return color;				
				break;
			}
		}
	}
	if(flag == 0)
	{
		return 0;
		imshow("src",src);
	} 
}
