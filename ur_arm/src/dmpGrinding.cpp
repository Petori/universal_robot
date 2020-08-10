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
string tool_frame = "tool0"; // 末端坐标系

// dmp迭代函数
Eigen::MatrixXd dmp_run_once(Eigen::MatrixXd previous_status, double goal, double tao, double dt, double wi[n_rfs]);
void goPose(Eigen::MatrixXd now_status_x, Eigen::MatrixXd now_status_y, Eigen::MatrixXd now_status_z, ur_arm::PoseMatrix pose_);
//获取机器人当前信息
void getRobotInfo(sensor_msgs::JointState curState);
//关节角控制方式，去起始位置
void goStartPose();
// 录轨迹数据用的测试位置
void goTestPose();

int main(int argc, char **argv)
{

  // 初始化
  ros::init(argc, argv, "dmpGrinding_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  ros::Subscriber joints_sub = node_handle.subscribe("/joint_states", 1, getRobotInfo);

  //去初始位置
  cout<<"去往初始姿态"<<endl;
  goStartPose();
  cout<<"到达初始姿态"<<endl;

  // cout<<"是否开始执行测试轨迹？确定请按ENTER 不确定请Ctrl+C"<<endl;
  // while(getchar()!='\n');
  // cout<<"开始执行测试轨迹"<<endl;
  // goTestPose();
  // cout<<"执行完毕"<<endl;

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
  double dt = 0.004;

  // 赋初值
  previous_status_x(0,0) = 1;
  previous_status_x(1,0) = startpos.p[0];
  previous_status_x(2,0) = 0;
  previous_status_x(3,0) = 0;

  previous_status_y(0,0) = 1;
  previous_status_y(1,0) = startpos.p[1];
  previous_status_y(2,0) = 0;
  previous_status_y(3,0) = 0;

  previous_status_z(0,0) = 1;
  previous_status_z(1,0) = startpos.p[2];
  previous_status_z(2,0) = 0;
  previous_status_z(3,0) = 0;


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
    t_int[i] = double(i)/(n_rfs-1);
    c[i] = exp(-1*alpha_x*t_int[i]);
    //cout<<"c[i]:"<<c[i]<<endl;
    if(i>0)
    {
      h[i-1]=(c[i]-c[i-1])*0.55;
    }
  }
  h[n_rfs]=h[n_rfs-1];

  //cout<<"previous_status: "<<previous_status(0,0)<<", "<<previous_status(1,0)<<", "<<previous_status(2,0)<<", "<<previous_status(3,0)<<endl;
  for(int i=0;i<n_rfs;i++)
  {
    psi(i,0) = exp(-0.5*(previous_status(0,0)-c[i])*(previous_status(0,0)-c[i])/(h[i]*h[i]));
    wii(i,0) = wi[i];
  }

  // cout<<"Psi: "<<endl;
  // cout<<psi<<endl;

  f = psi.transpose()*wii*previous_status(0,0)/psi.sum();
  f_value = f(0,0);
  //cout<<"f_value: "<<f_value<<endl;

  now_status(3,0) = alpha_y*(beta_y*(goal-previous_status(1,0))-previous_status(2,0)) + f_value;
  now_status(2,0) = previous_status(2,0) + tao*now_status(3,0)*dt;
  now_status(1,0) = previous_status(1,0) + tao*now_status(2,0)*dt;
  now_status(0,0) = previous_status(0,0) - tao*alpha_x*previous_status(0,0)*dt;
  //cout<<"now_status: "<<now_status(0,0)<<", "<<now_status(1,0)<<", "<<now_status(2,0)<<", "<<now_status(3,0)<<endl;

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

  cout<<"当前末端位置为: "<<pose_.p[0]<<", "<<pose_.p[1]<<", "<<pose_.p[2]<<endl;
  cout<<"现在去往目标位置："<<goPose_.position.x<<", ";
  cout<<goPose_.position.y<<", "<<goPose_.position.z<<endl;

  arm_group.setPoseTarget(goPose_,tool_frame);

  // targetWayPoints.push_back(goPose_);
  // arm_group.computeCartesianPath(targetWayPoints,
  //                                0.02,  // eef_step
  //                                0.0,   // jump_threshold
  //                                trajectory);
  // move_plan.trajectory_ = trajectory;

  arm_group.move();

}

void goStartPose()
{
  std::vector<double> sAng;
  moveit::planning_interface::MoveGroupInterface arm_group("manipulator");

  sAng.clear();
  // 位置是[x y z] = [0.2616 -0.3663 0.1032]
  sAng.push_back(1.95);
  sAng.push_back(-1.31);
  sAng.push_back(2.24);
  sAng.push_back(-2.5);
  sAng.push_back(-1.59);
  sAng.push_back(-0.24);

  arm_group.setJointValueTarget(sAng);
  arm_group.move();
}

void goTestPose()
{
  std::vector<double> sAng2;
  moveit::planning_interface::MoveGroupInterface arm_group("manipulator");

  sAng2.clear();
  // 位置是[x y z] = [0.3616 -0.0663 0.5032]
  sAng2.push_back(2.7653);
  sAng2.push_back(-1.6266);
  sAng2.push_back(1.4359);
  sAng2.push_back(-1.3692);
  sAng2.push_back(-1.5834);
  sAng2.push_back(0.5754);

  arm_group.setJointValueTarget(sAng2);
  arm_group.setMaxVelocityScalingFactor(0.05);// 慢点，方便录制
  arm_group.move();
}
