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

//#define sim_exp 1  //  仿真运行时取消注释此行
#define real_exp 1  //  实物运行时取消注释此行
#define n_rfs 30 // 基函数个数
sensor_msgs::JointState urState;  // 机械臂当前状态
const double coord_bias[3] = {0, 0.06, 0.325};   // 相机测量误差
int solutionNum = 8;    // 反解解系列
double tao = 1;
double dt = 0.0013;      // dmp迭代步长——与学习时的dt取同样的值
double timeIncrease = 20;

// dmp迭代函数
Eigen::MatrixXd dmp_run_once(Eigen::MatrixXd previous_status, double goal, double tao, double dt, double wi[n_rfs]);
void goPose(Eigen::MatrixXd now_status_x, Eigen::MatrixXd now_status_y, Eigen::MatrixXd now_status_z, ur_arm::PoseMatrix pose_);
//获取机器人当前信息
void getRobotInfo(const sensor_msgs::JointState& curState);
// 纠正关节角的排列顺序
sensor_msgs::JointState modifyJointState(sensor_msgs::JointState jS);
// 求空间三点的圆心
Eigen::MatrixXd solveCircleCenter(Eigen::MatrixXd point3d);
// 检查路径
bool checkPath(control_msgs::FollowJointTrajectoryGoal path);

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

  #ifdef sim_exp
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ur_control("/arm_controller/follow_joint_trajectory/", true);
  #endif

  #ifdef real_exp
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ur_control("/follow_joint_trajectory/", true);
  #endif

  ur_control.waitForServer();

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions={-1.1346,-1.2644,-2.2733,-2.7455,-1.1346,0};
  point.velocities={0, 0, 0, 0, 0, 0};
  point.time_from_start=ros::Duration(3.0);
  goal.trajectory.points.push_back(point);

  ur_control.sendGoal(goal);
  ur_control.waitForResult();

  // 从文件读取识别的三个靶标点
  ifstream fin_3b("/home/petori/data/parameter/detect_circles.txt");
  cout<<"读入三个靶标点..."<<endl;
  Eigen::MatrixXd point3b(3,3);
  char tmp1;
  double tmp2;
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<3;j++)
    {
      fin_3b>>tmp1;
      fin_3b>>tmp2;
      point3b(i,j) = tmp2;
    }
    fin_3b>>tmp1;
  }
  cout<<"三个靶标点："<<endl<<point3b<<endl;

  // 计算三个靶标点构成的圆心,并补偿相机测量误差
  Eigen::MatrixXd centerP(3,1);
  double goal_pos[3];

  centerP = solveCircleCenter(point3b);
  cout<<"圆心坐标："<<endl<<centerP.transpose()<<endl;

  goal_pos[0] = centerP(0,0) + coord_bias[0];
  goal_pos[1] = centerP(1,0) + coord_bias[1];
  goal_pos[2] = centerP(2,0) + coord_bias[2];

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

  // 暂停等待确认
  cout<<"是否执行加工？确定请按ENTER 不确定请Ctrl+C"<<endl;
  while(getchar()!='\n');

  // 获取当前机器人末端的笛卡尔位置和姿态
  std::vector<double> startang;
  ur_arm::PoseMatrix startpos;
  sensor_msgs::JointState cccState;

  //ros::spinOnce();
  cccState = urState;

  #ifdef sim_exp
  cccState = modifyJointState(cccState);
  #endif

  startang = cccState.position;
  startpos = fKine(startang);

  //cout<<"startpos: "<<endl<<startpos<<endl;

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

  // 定义目标
  control_msgs::FollowJointTrajectoryGoal path_goal;
  path_goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  double tnow;

  while((ros::ok())&&(previous_status_x(0,0)>1e-4))
  {
    //cout<<"previous_status: "<<previous_status_x(0,0)<<", "<<"位置: "<<"["<<previous_status_x(1,0)<<", "<<previous_status_y(1,0)<<", "<<previous_status_z(1,0)<<"]";
    //cout<<", "<<"速度: "<<"["<<previous_status_x(2,0)<<", "<<previous_status_y(2,0)<<", "<<previous_status_z(2,0)<<"]"<<endl;
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

    #ifdef sim_exp
    tmpState = modifyJointState(tmpState);// 实物不需要这行
    #endif

    tmpPose = fKine(tmpState.position);

    goalPose = tmpPose;
    goalPose.p[0] = now_status_x(1,0);
    goalPose.p[1] = now_status_y(1,0);
    goalPose.p[2] = now_status_z(1,0);
    //cout<<"goalPose: "<<endl<<goalPose<<endl;

    ur_arm::AllAng aAng;
    aAng = invKine(goalPose);
    if(aAng.ang8[6]==0)
    {
      cout<<"反解遇到问题，程序中止..."<<endl;
      return 0;
    }
    // 利用速度雅可比计算下一状态的关节速度
    std::vector<double> goal_joint_pos;//每次都重新定义，就不用clear()了吧？
    // std::vector<double> goal_joint_vel;
    // Eigen::MatrixXf g_jo_vel(6,1);
    // Eigen::MatrixXf g_car_vel(6,1);
    // Eigen::MatrixXf tmpJaco(6,6);

    goal_joint_pos.assign(aAng.ang8.begin(),aAng.ang8.end()); // 关节角赋值
    goal_joint_pos.pop_back(); // 删掉标志位
    // tmpJaco = cacJacob(goal_joint_pos);
    // g_car_vel(0,0) = now_status_x(2,0);
    // g_car_vel(1,0) = now_status_y(2,0);
    // g_car_vel(2,0) = now_status_z(2,0);
    // g_jo_vel = tmpJaco.inverse()*g_car_vel;
    //
    // for(int i=0;i<6;i++)
    // {
    //   goal_joint_vel.push_back(g_jo_vel(i,0));
    // }

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = goal_joint_pos;
    //point.velocities = goal_joint_vel; // 不用速度更安全
    point.velocities = {0, 0, 0, 0, 0, 0};
    //point.accelerations = {0, 0, 0, 0, 0, 0};

    tnow = tnow + dt*timeIncrease;
    point.time_from_start=ros::Duration(tnow);
    path_goal.trajectory.points.push_back(point);
    //cout<<"第一组关节角: "<<endl<<point<<endl;
    //cout<<"["<<goal_joint_pos[0]<<","<<goal_joint_pos[1]<<","<<goal_joint_pos[2]<<",";
    //cout<<goal_joint_pos[3]<<","<<goal_joint_pos[4]<<","<<goal_joint_pos[5]<<"]"<<endl;
    //return 0;
  }
  //return 0;
  // 执行目标
  if(checkPath(path_goal))
  {
    ur_control.sendGoal(path_goal);
    ur_control.waitForResult();
    cout<<"运动执行完毕。"<<endl;
  }
  else
  {
    cout<<"路径有问题，程序终止。"<<endl;
  }

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

void getRobotInfo(const sensor_msgs::JointState& curState)
{
  urState = curState;
  //cout<<"hi"<<endl;
}

sensor_msgs::JointState modifyJointState(sensor_msgs::JointState jc)
{
  sensor_msgs::JointState jS;
  double tmp;

  jS = jc;

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

Eigen::MatrixXd solveCircleCenter(Eigen::MatrixXd point3d)
{
  Eigen::MatrixXd centerpoint(3,1);
  double a1, b1, c1, d1;
  double a2, b2, c2, d2;
  double a3, b3, c3, d3;

  double x1 = point3d(0,0); double y1 = point3d(0,1); double z1 = point3d(0,2);
  double x2 = point3d(1,0); double y2 = point3d(1,1); double z2 = point3d(1,2);
  double x3 = point3d(2,0); double y3 = point3d(2,1); double z3 = point3d(2,2);

  a1 = (y1*z2 - y2*z1 - y1*z3 + y3*z1 + y2*z3 - y3*z2);
  b1 = -(x1*z2 - x2*z1 - x1*z3 + x3*z1 + x2*z3 - x3*z2);
  c1 = (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2);
  d1 = -(x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1);

  a2 = 2 * (x2 - x1);
  b2 = 2 * (y2 - y1);
  c2 = 2 * (z2 - z1);
  d2 = x1 * x1 + y1 * y1 + z1 * z1 - x2 * x2 - y2 * y2 - z2 * z2;

  a3 = 2 * (x3 - x1);
  b3 = 2 * (y3 - y1);
  c3 = 2 * (z3 - z1);
  d3 = x1 * x1 + y1 * y1 + z1 * z1 - x3 * x3 - y3 * y3 - z3 * z3;

  centerpoint(0,0) = -(b1*c2*d3 - b1*c3*d2 - b2*c1*d3 + b2*c3*d1 + b3*c1*d2 - b3*c2*d1)
      /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
  centerpoint(1,0) =  (a1*c2*d3 - a1*c3*d2 - a2*c1*d3 + a2*c3*d1 + a3*c1*d2 - a3*c2*d1)
      /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);
  centerpoint(2,0) = -(a1*b2*d3 - a1*b3*d2 - a2*b1*d3 + a2*b3*d1 + a3*b1*d2 - a3*b2*d1)
      /(a1*b2*c3 - a1*b3*c2 - a2*b1*c3 + a2*b3*c1 + a3*b1*c2 - a3*b2*c1);

  return centerpoint;
}

bool checkPath(control_msgs::FollowJointTrajectoryGoal path)
{
  trajectory_msgs::JointTrajectory traj;
  trajectory_msgs::JointTrajectoryPoint point_tmp;
  std::vector<double> anglast;
  std::vector<double> angnow;
  double dis;
  bool status = true;

  traj = path.trajectory;
  point_tmp = traj.points.at(traj.points.size()-1);
  traj.points.pop_back();
  anglast = point_tmp.positions;

  #ifdef sim_exp
  double tmp1;
  tmp1 = anglast[0];anglast[0] = anglast[2];anglast[2] = tmp1;
  #endif

  for(size_t i=1;i<traj.points.size();i++)
  {
    dis = 0;
    point_tmp = traj.points.at(traj.points.size()-1);
    traj.points.pop_back();
    angnow.clear();
    angnow = point_tmp.positions;

    #ifdef sim_exp
    double tmp2;
    tmp2 = angnow[0];angnow[0] = angnow[2];angnow[2] = tmp2;
    #endif

    cout<<"[";
    for(int j=0;j<6;j++)
    {
      dis = fabs(angnow[j] - anglast[j]);
      if(dis>0.2)
      {
        status = false;
      }
      cout<<anglast[j]<<",";
    }
    cout<<"]"<<endl;
    anglast.clear();
    anglast = angnow;
  }
  return status;
}
