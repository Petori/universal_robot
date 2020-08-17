// // Created by Petori on 2020/7/22

/*--------------------------------------------------------测试一--------------------------------*/
// // C++基础
// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <string>
// #include <math.h>
// #include <Eigen/Dense>
// #include <unistd.h>   // for function usleep(microseconds)
// #include <cstdlib>
// #include <signal.h>   // deal with the "ctrl + C"
// #include <queue>
// // ROS基础
// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/WrenchStamped.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <sensor_msgs/JointState.h>
// #include <std_msgs/Int8.h>
// #include "ur_arm/Joints.h"
// #include <moveit/move_group_interface/move_group_interface.h>   // replace the old version "move_group.h"
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <tf2_ros/transform_listener.h>
// // 单列
// #include "ur_arm/my_func.h"
// #include "netft_utils/SetBias.h"
//
// using namespace std;
//
//
//
// int main(int argc, char **argv)
// {
//
//   // 测试函数urarmPose_2_geomePose和geomePose_2_urarmPose
//   // ur_arm::PoseMatrix test_u;
//   // geometry_msgs::Pose test_g;
//   // ur_arm::PoseMatrix test_r;
//   //
//   // test_u.n[0] = 1;test_u.n[1] = 0;test_u.n[2] = 0;
//   // test_u.o[0] = 0;test_u.o[1] = 1;test_u.o[2] = 0;
//   // test_u.a[0] = 0;test_u.a[1] = 0;test_u.a[2] = 1;
//   // test_u.p[0] = 0.2;test_u.p[1] = 0.3;test_u.p[2] = 0.4;
//   //
//   //
//   // test_g = urarmPose_2_geomePose(test_u);
//   // test_r = geomePose_2_urarmPose(test_g);
//   //
//   // cout<<"before:"<<endl;
//   // showPoseMatrix(test_u);
//   // cout<<"after"<<endl;
//   // showPoseMatrix(test_r);
//
//   // 测试函数rot2rpy
//   // std::vector<double> position;
//   // ur_arm::PoseMatrix pose;
//   // std::vector<double> rpy;
//   //
//   // for(int i=0;i<6;i++)
//   // {
//   //   position.push_back(1);
//   // }
//   // pose = fKine(position);
//   // rpy = rot2rpy(pose);
//   //
//   // cout<<"rpy: "<<rpy[0]<<", "<<rpy[1]<<", "<<rpy[2]<<endl;
//
//   return 0;
// }


/*--------------------------------------------------------测试二--------------------------------*/
#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <std_msgs/Int8.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

Mat img_color;		        //相机彩图
bool receive_img=false;	  //是否收到图像

void rosimage2opencv(const sensor_msgs::ImageConstPtr msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception &e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	//cvtColor(cv_ptr->image,I_ORI,CV_BGR2GRAY);
	img_color=cv_ptr->image;
	receive_img=true;
  cv::imshow("rgb_image",img_color);
  waitKey(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "eye_to_hand_calculate");   //定义节点名称
	ros::NodeHandle handle;   //为这个进程的节点创建一个句柄
  ros::AsyncSpinner spinner(3);
  spinner.start();

	//接收话题
	ros::Subscriber image_sub=handle.subscribe("/camera/color/image_raw",1,rosimage2opencv); //订阅相机图像，并转换为Mat格式

  sleep(5);
	return 0;
}
