// 2018/06/19——机器人关节2或关节3慢速运动
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<ur_arm::Joints>("/ur_arm/cmd_joint_vel", 1);
  sleep(2.0);
  ur_arm::Joints vel_move;
  ur_arm::Joints vel_static;

  vel_move.base = 0;
  vel_move.shoulder =0.1;
  vel_move.elbow = 0.0;
  vel_move.wrist1 = 0.0;
  vel_move.wrist2 = 0.0;
  vel_move.wrist3 = 0.0;

  vel_static.base = 0;
  vel_static.shoulder = 0.0;
  vel_static.elbow = 0;
  vel_static.wrist1 = 0;
  vel_static.wrist2 = 0;
  vel_static.wrist3 = 0;

  chatter_pub.publish(vel_move);
  sleep(20);
  chatter_pub.publish(vel_static);

  ROS_INFO("The robot will stop from now.");
  chatter_pub.publish(vel_static);
  ROS_INFO("Stopped.");

  return 0;
}
// 2018/06/19备份——机器人单关节阶跃响应
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<ur_arm::Joints>("/ur_arm/cmd_joint_vel", 1);
  sleep(2.0);
  ur_arm::Joints vel_move;
  ur_arm::Joints vel_static;
  ur_arm::Joints vel_back;

  vel_move.base = 0;
  vel_move.shoulder =0;
  vel_move.elbow = 0.6;
  vel_move.wrist1 = 0.0;
  vel_move.wrist2 = 0.0;
  vel_move.wrist3 = 0.0;

  vel_static.base = 0;
  vel_static.shoulder = 0.0;
  vel_static.elbow = 0;
  vel_static.wrist1 = 0;
  vel_static.wrist2 = 0;
  vel_static.wrist3 = 0;

  vel_back.base = 0;
  vel_back.shoulder = 0;
  vel_back.elbow = 0.6;
  vel_back.wrist1 = 0;
  vel_back.wrist2 = 0;
  vel_back.wrist3 = 0;

  chatter_pub.publish(vel_move);
  sleep(2);
  chatter_pub.publish(vel_static);
  sleep(2);
  chatter_pub.publish(vel_back);
  sleep(2);
  chatter_pub.publish(vel_static);
  sleep(2);
//  chatter_pub.publish(vel_move);
//  sleep(2);
//  chatter_pub.publish(vel_static);
//  sleep(2);
//  chatter_pub.publish(vel_back);
//  sleep(2);

  ROS_INFO("The robot will stop from now.");
  chatter_pub.publish(vel_static);
  ROS_INFO("Stopped.");

  return 0;
}
