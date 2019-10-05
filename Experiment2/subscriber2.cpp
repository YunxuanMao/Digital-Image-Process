#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define LINEAR_X 0
#define ROWS 256
#define COLS 512

using namespace cv;

class Comp{
public:
    Comp(double r = 0.0, double i = 0.0): real(r), imag(i) {};
    Comp operator+ (const Comp &c2) const;
    Comp operator- (const Comp &c2) const;
    Comp operator* (const Comp &c2) const;
    Comp operator/ (const Comp &c2) const;
    double spec();
    Comp conj();

public:
    double real;
    double imag;
};

Comp Comp::operator+ (const Comp &c2) const{
    return Comp(real + c2.real, imag + c2.imag);
}
Comp Comp::operator- (const Comp &c2) const{
    return Comp(real - c2.real, imag - c2.imag);
}
Comp Comp::operator* (const Comp &c2) const{
    return Comp(real*c2.real - imag*c2.imag, real*c2.imag + imag*c2.real);
}
Comp Comp::operator/ (const Comp &c2) const{
    return Comp((real*c2.real + imag*c2.imag)/(c2.imag*c2.imag + c2.real*c2.real), (imag*c2.real - real*c2.imag)/(c2.imag*c2.imag + c2.real*c2.real));
}
double Comp::spec(){
    return sqrt(real*real + imag*imag);
}
Comp Comp::conj(){
    Comp c1(real, -imag);
    return c1;
}
//////////////////////滤波//////////////////
// 空域高斯滤波器函数
void Gaussian(Mat input, Mat output,const double (*sigma)[3] , double b);
// 快速傅里叶变换
void fastFuriorTransform(Mat output, Comp output_fft[][COLS]);
Comp WN(double n, double N);
void fft(int M, Comp *input, bool inv);
void fft2(int M, int N, Comp (*input)[COLS], bool inv);
// 理想低通滤波器函数
//Mat ideal_lbrf_kernel(Mat scr,float sigma);
// 频率域滤波函数
// src:原图像
// blur:滤波器函数
//Mat freqfilt(Mat scr,Mat blur);
//////////////////////形态学//////////////////
// 膨胀函数
//void Dilate(Mat Src, Mat Tem, Mat Dst);
// 腐蚀函数
//void Erode(Mat Src, Mat Tem, Mat Dst);

int main(int argc, char **argv)
{
    VideoCapture capture;
    capture.open(0);//打开 zed 相机
    ROS_WARN("*****START");
    ros::init(argc,argv,"trafficLaneTrack");//初始化 ROS 节点
    ros::NodeHandle n;
    // ros::Rate loop_rate(10);//定义速度发布频率
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5);//定义速度发布器
    if (!capture.isOpened())
    {
        printf("摄像头没有正常打开,重新插拔工控机上当摄像头\n");
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
        Mat frIn = frame(cv::Rect(0, 0, COLS, ROWS));//使用笔记本摄像头

        // Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取 zed 的左目图片
        Mat frIn_grey = cv::Mat(frIn.rows,frIn.cols,CV_8U);
        cv::cvtColor(frIn,frIn_grey,CV_BGR2GRAY);
        Mat figure1 = cv::Mat(frIn.rows,frIn.cols,CV_8U);
        Mat figure2 = cv::Mat(frIn.rows,frIn.cols,CV_8U);
        // 空域滤波函数
        double blur1[3][3] = {{-1,-1,-1},{-1,8,-1},{-1,-1,-1}};// 滤波器
        Gaussian(frIn_grey,figure1,blur1,1);
        // 快速傅里叶变换
        Comp output_fft[ROWS][COLS];
        figure2 = frIn_grey.clone();
        fastFuriorTransform(figure2, output_fft);
        // 频域滤波函数
        //freqfilt();
        // 膨胀函数
        //Dilate();
        // 腐蚀函数
        //Erode();
        imshow("Origin",frIn);
        imshow("Grey",frIn_grey);
        imshow("Gaussian",figure1);
        imshow("FFT",figure2);
        ros::spinOnce();
        waitKey(5);
    }
    return 0;
}

//////////////////////滤波//////////////////
// 空域高斯滤波器函数
// input：原图像
// output：处理后图像
// sigma：滤波器矩阵
void Gaussian(const Mat input, Mat output,const double (*sigma)[3] , double b)
{
    double a = 0;
    for(int i = 1;i < input.rows - 1;i++){
        for(int j = 1;j < input.cols - 1;j++){
            for(int k = 0;k < 3;k++){
                for(int l = 0;l < 3;l++){
                    a += sigma[k][l]*input.at<uchar>(i+k-1,j+l-1);
                }
            }
            output.at<uchar>(i,j) = a/b;
            a = 0;
        }
    }
}
// 快速傅里叶变换
void fastFuriorTransform(Mat output, Comp output_fft[][COLS]) 
{
    int M = output.rows, N = output.cols;
    for(int i = 0;i < M;i++){
        for(int j = 0;j < N;j++){
            output_fft[i][j].real = double(output.at<uchar>(i,j))*pow((-1),(i+j));
            output_fft[i][j].imag = 0;
        }
    }
    fft2(M, N, output_fft, false);
    for(int i = 0;i < M;i++){
        for(int j = 0;j < N;j++){
            output.at<uchar>(i,j) = log(1 + output_fft[i][j].spec())/log(1.07);
            //if(log(1 + output_fft[i][j].spec())/log(1.07)>255) printf("%f\n",log(1 + output_fft[i][j].spec())/log(1.07));
            //printf("%f\n",log(1 + output_fft[i][j].spec())/log(1.1));
        }
    }
}
Comp WN(double n, double N){
    Comp a(cos(-2*M_PI*n/N), sin(-2*M_PI*n/N));
    return a;
}
void fft(int M, Comp *input, bool inv = false){//if inv == true, 逆变换
    if(M == 1) return;
    static Comp buf[COLS]; 
    int k = M/2;
    for(int i = 0;i < k;i++){
        buf[i] = input[2*i];
        buf[i + k] = input[2*i + 1];
    }
    for(int i = 0;i < M;i++){
        input[i] = buf[i];
    }

    fft(k, input, inv);
    fft(k, input + k, inv);
    
    for(int i = 0;i < k;i++){
        Comp x = WN(i,M);
        if(inv) x = x.conj();
        buf[i] = input[i] + x*input[i + k];
        buf[i + k] = input[i] - x*input[i + k];
    }
    for(int i = 0;i < M;i++){
        input[i] = buf[i];
    }
    
}

void fft2(int M, int N, Comp (*input)[COLS], bool inv){
    
    for(int i = 0;i < M;i++){
        fft(N, *(input + i), inv);
    }
    Comp a[M];
    for(int i = 0;i < N;i++){
        for(int j = 0;j < M;j++){
            a[j].real = input[j][i].real;
            a[j].imag = input[j][i].imag;
        }
        fft(M, a, inv);
        for(int j = 0;j < M;j++){
            input[j][i].real = a[j].real;
            input[j][i].imag = a[j].imag;
        }
    }
}
// 理想低通滤波器函数
//Mat ideal_lbrf_kernel(Mat scr,float sigma)
// 频率域滤波函数
// src:原图像
// blur:滤波器函数
//Mat freqfilt(Mat scr,Mat blur)
//////////////////////形态学//////////////////
// 膨胀函数
//void Dilate(Mat Src, Mat Tem, Mat Dst)
// 腐蚀函数
//void Erode(Mat Src, Mat Tem, Mat Dst)