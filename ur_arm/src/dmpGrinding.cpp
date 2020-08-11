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
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
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
// 纠正关节角的排列顺序
sensor_msgs::JointState modifyJointState(sensor_msgs::JointState jS);

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
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ur_control("/arm_controller/follow_joint_trajectory/", true);

  ROS_INFO("Waiting for action server to start.");
  ur_control.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions={1.95,-1.31,2.24,-2.5,-1.59,-0.24};
  point.velocities={0, 0, 0, 0, 0, 0};
  point.time_from_start=ros::Duration(5.0);
  goal.trajectory.points.push_back(point);

  ur_control.sendGoal(goal);

  // 函数测试——jointVel_2_cartesianVel（）
  // sleep(1);
  // cout<<"函数测试"<<endl<<endl;;
  // ur_arm::cartesianState cccStates;
  // sensor_msgs::JointState jjjStates;
  // jjjStates = urState;
  // cccStates = jointVel_2_cartesianVel(jjjStates);
  //
  // cout<<"关节状态: "<<endl;
  // cout<<"位置: ";
  // for(int i=0;i<6;i++)
  // {
  //   cout<<jjjStates.position[i]<<",";
  // }
  // cout<<endl;
  // cout<<"速度: ";
  // for(int i=0;i<6;i++)
  // {
  //   cout<<jjjStates.velocity[i]<<",";
  // }
  // cout<<endl<<endl;
  //
  // cout<<"末端状态: "<<endl;
  // cout<<"位置: ";
  // for(int i=0;i<6;i++)
  // {
  //   cout<<cccStates.position[i]<<",";
  // }
  // cout<<endl;
  // cout<<"速度";
  // for(int i=0;i<6;i++)
  // {
  //   cout<<cccStates.velocity[i]<<",";
  // }
  // cout<<endl<<endl;
  //测试结束

  ur_control.waitForResult();

  return 0;

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

    // 获取当前位姿保证姿态不变，反解求取下一状态关节角
    sensor_msgs::JointState tmpState;
    ur_arm::PoseMatrix tmpPose;// 当前位姿
    ur_arm::PoseMatrix goalPose;//目标位姿


    tmpState = urState;
    tmpState = modifyJointState(tmpState);// 实物不需要这行
    tmpPose = fKine(tmpState.position);

    goalPose = tmpPose;
    goalPose.p[0] = now_status_x(1,0);
    goalPose.p[1] = now_status_y(1,0);
    goalPose.p[2] = now_status_z(1,0);

            // 此段判断解系（先不写）
    int solutionNum = 3;
            // 判断结束

    ur_arm::AllAng aAng;
    aAng = invKine(goalPose);
    if(aAng.ang3[6]==0)
    {
      cout<<"反解遇到问题，程序中止..."<<endl;
      while(true);
    }

    // 利用速度雅可比计算下一状态的关节速度
    std::vector<double> goal_joint_pos;//每次都重新定义，就不用clear()了吧？
    std::vector<double> goal_joint_vel;
    Eigen::MatrixXf g_jo_vel(6,1);
    Eigen::MatrixXf g_car_vel(6,1);
    Eigen::MatrixXf tmpJaco(6,6);

    goal_joint_pos.assign(aAng.ang3.begin(),aAng.ang3.end()); // 关节角赋值
    goal_joint_pos.pop_back(); // 删掉标志位

    tmpJaco = cacJacob(goal_joint_pos);
    g_car_vel(0,0) = now_status_x(2,0);
    g_car_vel(1,0) = now_status_y(2,0);
    g_car_vel(2,0) = now_status_z(2,0);
    g_jo_vel = tmpJaco.inverse()*g_car_vel;
    for(int i=0;i<6;i++)
    {
      goal_joint_vel[i] = g_jo_vel(i,0);
    }

    // 执行目标
    double dt;//时间间隔，注意
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = goal_joint_pos;
    point.velocities = goal_joint_vel;
    //point.accelerations = {0, 0, 0, 0, 0, 0};

    point.time_from_start=ros::Duration(dt);
    goal.trajectory.points.push_back(point);
    ur_control.sendGoal(goal);
    ur_control.waitForResult();

    // 利用urState给当前状态赋值——加速度不管
    ur_arm::cartesianState carState;
    tmpState = urState;
    tmpState = modifyJointState(tmpState);// 实物不需要这行

    carState = jointVel_2_cartesianVel(tmpState);
    previous_status_x(1,0) = carState.position[0];
    previous_status_y(1,0) = carState.position[1];
    previous_status_z(1,0) = carState.position[2];
    previous_status_x(2,0) = carState.velocity[0];
    previous_status_y(2,0) = carState.velocity[1];
    previous_status_z(2,0) = carState.velocity[2];
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

sensor_msgs::JointState modifyJointState(sensor_msgs::JointState jc)
{
  sensor_msgs::JointState jS;
  double tmp;

  tmp = jS.position[0];
  jS.position[0] = jS.position[2];
  jS.position[2] = tmp;

  tmp = jS.velocity[0];
  jS.velocity[0] = jS.velocity[2];
  jS.velocity[2] = tmp;

  tmp = jS.effort[0];
  jS.effort[0] = jS.effort[2];
  jS.effort[2] = tmp;

  return jS;
}
