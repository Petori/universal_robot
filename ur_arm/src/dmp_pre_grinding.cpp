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

int main(int argc, char **argv)
{

  // 初始化
  ros::init(argc, argv, "dmp_pre_grinding_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(3);
  spinner.start();

  //去初始位置
  cout<<"去往初始姿态"<<endl;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ur_control("/follow_joint_trajectory/", true);

  ur_control.waitForServer();

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions={-1.9009,-1.6507,-1.8012,-2.8312,0.3301,0};
  point.velocities={0, 0, 0, 0, 0, 0};
  point.time_from_start=ros::Duration(2.0);
  goal.trajectory.points.push_back(point);

  ur_control.sendGoal(goal);
  ur_control.waitForResult();

  cout<<"是否执行加工？确定请按ENTER 不确定请Ctrl+C"<<endl;
  while(getchar()!='\n');

  // 点到点路径
  control_msgs::FollowJointTrajectoryGoal goal1;
  goal1.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  trajectory_msgs::JointTrajectoryPoint point1;
  point1.positions={-1.1375,   -2.0625,   -1.7331 ,  -2.4876  ,  1.0935 ,   0.0001};
  point1.velocities={0, 0, 0, 0, 0, 0};
  point1.time_from_start=ros::Duration(5.0);
  goal1.trajectory.points.push_back(point1);

  ur_control.sendGoal(goal1);
  ur_control.waitForResult();
  cout<<"运动执行完毕。"<<endl;

  return 0;
}
