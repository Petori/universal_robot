// Created by Petori on 2020/8/14
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <fstream>
#include <vector>
// 相机相关
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// 全局变量
Mat mat_image_rgb;
Mat mat_image_depth;
bool show_image=true;
int minR = 0; // 最小圆半径
int maxR = 50; // 最大圆半径
//realsense相机话题
std::string image_rgb_str =  "camera/color/image_raw";
std::string image_depth_str = "camera/depth/image_rect_raw";
//realsense camera parameter
double camera_factor = 1000;
double camera_cx = 638.9251668336952;
double camera_cy = 361.8208262607499;
double camera_fx = 910.1410035061172;
double camera_fy = 910.2328790906075;


// 函数声明
void imageCallback_depth( const sensor_msgs::ImageConstPtr  &image_depth);
void imageCallback_color( const sensor_msgs::ImageConstPtr &image_rgb);
std::vector<Point3f> GetPointCoord(std::vector<Point2f>& centers, cv::Mat image_rgb, cv::Mat image_depth);
vector<Point2f> hough(Mat img);          // 霍夫圆检测

int main(int argc, char **argv)
{
  ros::init(argc, argv, "circleGoalDetect");   //定义节点名称
  ros::NodeHandle handle;   //为这个进程的节点创建一个句柄

  ros::Subscriber sub1 = handle.subscribe(image_rgb_str, 1, imageCallback_color);
  ros::Subscriber sub2 = handle.subscribe(image_depth_str, 1, imageCallback_depth);

  cout<<"开始检测？按ENTER继续，按CTRL+C退出"<<endl;
  while(getchar()!='\n');
  cout<<"继续...."<<endl;

  // 回调一次，拍摄一张图片
  ros::spinOnce();

  // 开始检测圆
  vector<Point2f> centers;
  vector<Point3f> centers_cam_coord;
  centers = hough(mat_image_rgb);// 得到彩图上的圆心像素坐标
  centers_cam_coord = GetPointCoord(centers, mat_image_rgb, mat_image_depth); // 得到相机坐标系下的所有圆心坐标[x,y,z]

  // 读入手眼关系
  ifstream fin_c2b("/home/petori/data/parameter/base2cam.txt");
  Mat base2cam(4, 4, CV_64FC1, Scalar::all(0));
  char tmp1;
  double tmp2;

  for(int i=0;i<4;i++)
  {
    for(int j=0;j<4;j++)
    {
      fin_c2b>>tmp1;
      fin_c2b>>tmp2;
      base2cam.at<double>(i,j) = tmp2;
    }
  }
  // cout<<base2cam<<endl; // 检查赋值

  // 转换圆心坐标到机器人坐标系下
  Mat pointHomo_c(4, 1, CV_64FC1, Scalar::all(0));
  Mat pointHomo_b(4, 1, CV_64FC1, Scalar::all(0));

  cout<<"开始写入圆心坐标"<<endl;
  ofstream fout("/home/petori/data/parameter/detect_circles.txt");
  for(size_t i=0;i<centers_cam_coord.size();i++)
  {
    // 逐点转换
    pointHomo_c.at<double>(0,0) = centers_cam_coord[i].x;
    pointHomo_c.at<double>(1,0) = centers_cam_coord[i].y;
    pointHomo_c.at<double>(2,0) = centers_cam_coord[i].z;
    pointHomo_c.at<double>(3,0) = 1;
    pointHomo_b = base2cam.inv()*pointHomo_c;

    // 将圆心坐标写入文件
    fout<<"["<<pointHomo_b.at<double>(0,0)<<", "<<pointHomo_b.at<double>(1,0)<<", "<<pointHomo_b.at<double>(2,0)<<"]"<<endl;
    cout<<"["<<pointHomo_b.at<double>(0,0)<<", "<<pointHomo_b.at<double>(1,0)<<", "<<pointHomo_b.at<double>(2,0)<<"]"<<endl;
  }
  cout<<"圆心坐标写入完毕"<<endl;
  return 0;
}

vector<Point2f> hough(Mat img)
{
  Mat gray;
  cvtColor(img, gray, COLOR_BGR2GRAY);
  //GaussianBlur(gray, gray, Size(3, 3), 2, 2);
  cv::imshow("Hough_circle_image", gray);
  waitKey(0);

  std::vector<Vec3f> circles;
  std::vector<Point2f> centers;

  //HoughCircles(gray, circles, HOUGH_GRADIENT, 2, gray.rows/10);
  HoughCircles(gray, circles, HOUGH_GRADIENT,1,10,100,30,5,25);

  Mat img_plot;// 在图像上画图并显示
  img_plot = mat_image_rgb;
  for(size_t i=0;i<circles.size();i++)
  {
    Point center(cvRound(circles[i][0]),cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    if((radius>minR)&(radius<maxR))
    {
      centers.push_back(center);
      cout<<"检测到的圆半径为："<<radius<<endl;
      circle(img_plot, center, radius,Scalar(0,255, 0), 2, 8, 0);// 画圆形
    }
  }
  cv::imshow("Hough_circle_image", img_plot);
  waitKey(0);
  return centers;
}

void imageCallback_depth( const sensor_msgs::ImageConstPtr  &image_depth )
{
    mat_image_depth = cv_bridge::toCvCopy(image_depth)->image;
}

void imageCallback_color( const sensor_msgs::ImageConstPtr &image_rgb)
{
  cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr=cv_bridge::toCvCopy(image_rgb,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	//cvtColor(cv_ptr->image,I_ORI,CV_BGR2GRAY);
	mat_image_rgb=cv_ptr->image;
  //cv::imshow("rgb_image", mat_image_rgb);
}

std::vector<Point3f> GetPointCoord(std::vector<Point2f>& centers, cv::Mat image_rgb, cv::Mat image_depth)
{
  std::vector<Point3f> pointCoords;
  Point3f tmp3;
  Point2f tmp2;

  for(size_t i = 0;i<centers.size();i++)
  {
    tmp2 = centers[i];
    int a = tmp2.x;
    int b = tmp2.y;

    ushort d = image_depth.ptr<ushort>(b)[a];
    // 计算这个点的空间坐标
    tmp3.z = double(d) / camera_factor;
    tmp3.x = (a- camera_cx) * tmp3.z / camera_fx;
    tmp3.y = (b - camera_cy) * tmp3.z / camera_fy;

    pointCoords.push_back(tmp3);
  }

  return pointCoords;
}
