// Created by Petori on 2020/7/22
// C++基础
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <unistd.h>   // for function usleep(microseconds)
#include <cstdlib>
#include <signal.h>   // deal with the "ctrl + C"
#include <queue>
// ROS基础
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"
#include <moveit/move_group_interface/move_group_interface.h>   // replace the old version "move_group.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <tf2_ros/transform_listener.h>
// 单列
#include "ur_arm/my_func.h"
#include "netft_utils/SetBias.h"

using namespace std;

#define n_rfs 30 // 基函数个数
sensor_msgs::JointState urState;  //机械臂当前状态

// dmp迭代函数
Eigen::MatrixXd dmp_run_once(Eigen::MatrixXd previous_status, double goal, double tao, double dt, double wi[n_rfs]);
void goPose(Eigen::MatrixXd now_status_x, Eigen::MatrixXd now_status_y, Eigen::MatrixXd now_status_z, ur_arm::PoseMatrix pose_);
//获取机器人当前信息
void getRobotInfo(sensor_msgs::JointState curState);

int main(int argc, char **argv)
{

  // 初始化
  ros::init(argc, argv, "dmpGrinding_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  ros::Subscriber joints_sub = node_handle.subscribe("/joint_states", 1, getRobotInfo);

  // 从文件读取圆心坐标
  ifstream fin("/home/petori/data/parameter/cc_pos.txt");
  cout<<"读入目标位置..."<<endl;
  double goal_pos[3];
  for(int i=0;i<3;i++)
  {
    fin>>goal_pos[i];
    cout<<goal_pos[i]<<endl;
  }

  // 从文件读取DMP参数
  ifstream fin_w("/home/petori/data/parameter/dmp_params.txt");
  cout<<"读入dmp学习参数..."<<endl;
  double wi_x[n_rfs];
  double wi_y[n_rfs];
  double wi_z[n_rfs];
  for(int i=0;i<(3*n_rfs);i++)
  {
    if(i<n_rfs)
    {
      fin_w>>wi_x[i];
    }
    else if(i<(2*n_rfs))
    {
      fin_w>>wi_y[i-n_rfs];
    }
    else
    {
      fin_w>>wi_z[i-2*n_rfs];
    }
  }
  cout<<"读入完毕。"<<endl;

  // 暂停等待确认
  cout<<"是否执行加工？确定请按ENTER 不确定请Ctrl+C"<<endl;
  while(getchar()!='\n');

  // 获取当前机器人末端的笛卡尔位置和姿态
  std::vector<double> startang;
  ur_arm::PoseMatrix startpos;

  startang = urState.position;
  startpos = fKine(startang);

  // 按笛卡尔空间拆分----每维单独一组dmp
  Eigen::MatrixXd now_status_x(4,1);
  Eigen::MatrixXd now_status_y(4,1);
  Eigen::MatrixXd now_status_z(4,1);
  Eigen::MatrixXd previous_status_x(4,1);
  Eigen::MatrixXd previous_status_y(4,1);
  Eigen::MatrixXd previous_status_z(4,1);

  double goal_x;
  double goal_y;
  double goal_z;

  goal_x = goal_pos[0];
  goal_y = goal_pos[1];
  goal_z = goal_pos[2];

  double tao = 1;
  double dt;

  while(now_status_x(0,0)>0)
  {
    now_status_x = dmp_run_once(previous_status_x,goal_x,tao,dt,wi_x);
    previous_status_x = now_status_x;
    now_status_y = dmp_run_once(previous_status_y,goal_y,tao,dt,wi_y);
    previous_status_y = now_status_y;
    now_status_z = dmp_run_once(previous_status_z,goal_z,tao,dt,wi_z);
    previous_status_z = now_status_z;
    // 执行目标
    goPose(now_status_x,now_status_y,now_status_z,startpos);
  }
  cout<<"运动执行完毕。"<<endl;

  return 0;
}

Eigen::MatrixXd dmp_run_once(Eigen::MatrixXd previous_status, double goal, double tao, double dt, double wi[n_rfs])
{
  // previous_status:[x_now, y, yd, ydd]
  // 定义核函数
  Eigen::MatrixXd interval(n_rfs,1);
  Eigen::MatrixXd psi(n_rfs,1);
  Eigen::MatrixXd wii(n_rfs,1);
  Eigen::MatrixXd f(1,1);
  Eigen::MatrixXd now_status(4,1);
  double t_int[n_rfs];
  double c[n_rfs];
  double h[n_rfs];
  double f_value;

  double alpha_y = 25;
  double beta_y;
  double alpha_x = 8;
  beta_y = alpha_y/4;

  for(int i=0;i<n_rfs;i++)
  {
    t_int[i] = i/(n_rfs-1);
    c[i] = exp(-1*alpha_x*t_int[i]);
    if(i>0)
    {
      h[i-1]=(c[i]-c[i-1])*0.55;
    }
  }
  h[n_rfs]=h[n_rfs-1];

  for(int i=0;i<n_rfs;i++)
  {
    psi(i,0) = exp(-0.5*(previous_status(0,0)-c[i])*(previous_status(0,0)-c[i])/(h[i]*h[i]));
    wii(i,0) = wi[i];
  }

  f = psi.transpose()*wii*previous_status(0,0)/psi.sum();
  f_value = f(0,0);

  now_status(3,0) = alpha_y*(beta_y*(goal-previous_status(1,0))-previous_status(2,0)) + f_value;
  now_status(2,0) = previous_status(2,0) + tao*now_status(3,0)*dt;
  now_status(1,0) = previous_status(1,0) + tao*now_status(2,0)*dt;
  now_status(0,0) = previous_status(0,0) - tao*alpha_x*previous_status(0,0)*dt;

  return now_status;
}

void getRobotInfo(sensor_msgs::JointState curState)
{
  urState = curState;
}

void goPose(Eigen::MatrixXd now_status_x, Eigen::MatrixXd now_status_y, Eigen::MatrixXd now_status_z, ur_arm::PoseMatrix pose_)
{
  moveit::planning_interface::MoveGroupInterface arm_group("manipulator");
  std::vector<geometry_msgs::Pose> targetWayPoints;
  moveit_msgs::RobotTrajectory trajectory;
  moveit::planning_interface::MoveGroupInterface::Plan move_plan;
  geometry_msgs::Pose goPose_;
  geometry_msgs::Pose mid_forrot;

  mid_forrot = urarmPose_2_geomePose(pose_);
  goPose_.orientation.w = mid_forrot.orientation.w;
  goPose_.orientation.x = mid_forrot.orientation.x;
  goPose_.orientation.y = mid_forrot.orientation.y;
  goPose_.orientation.z = mid_forrot.orientation.z;

  goPose_.position.x = now_status_x(1,0);
  goPose_.position.y = now_status_y(1,0);
  goPose_.position.z = now_status_z(1,0);

  targetWayPoints.push_back(goPose_);
  arm_group.computeCartesianPath(targetWayPoints,
                                 0.02,  // eef_step
                                 0.0,   // jump_threshold
                                 trajectory);
  move_plan.trajectory_ = trajectory;

  arm_group.execute(move_plan);

}
