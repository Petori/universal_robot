// 2018/06/19——机器人关节2或关节3慢速运动
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <unistd.h>   // for function usleep(microseconds)
#include "ur_arm/Joints.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<ur_arm::Joints>("/ur_arm/cmd_joint_vel", 1);
  sleep(2.0);
  ur_arm::Joints vel_moveF_joint1;
  ur_arm::Joints vel_moveB_joint1;
  ur_arm::Joints vel_move_joint1;
  ur_arm::Joints vel_moveF_joint2;
  ur_arm::Joints vel_moveB_joint2;
  ur_arm::Joints vel_move_joint2;
  ur_arm::Joints vel_stop;

  vel_moveF_joint1.base = 0;
  vel_moveF_joint1.shoulder = 2;
  vel_moveF_joint1.elbow = 0.0;
  vel_moveF_joint1.wrist1 = 0.0;
  vel_moveF_joint1.wrist2 = 0.0;
  vel_moveF_joint1.wrist3 = 0.0;

  vel_moveB_joint1.base = 0;
  vel_moveB_joint1.shoulder = -2;
  vel_moveB_joint1.elbow = 0.0;
  vel_moveB_joint1.wrist1 = 0.0;
  vel_moveB_joint1.wrist2 = 0.0;
  vel_moveB_joint1.wrist3 = 0.0;

  vel_moveF_joint2.base = 0;
  vel_moveF_joint2.shoulder = 0.0;
  vel_moveF_joint2.elbow = 2;
  vel_moveF_joint2.wrist1 = 0.0;
  vel_moveF_joint2.wrist2 = 0.0;
  vel_moveF_joint2.wrist3 = 0.0;

  vel_moveB_joint2.base = 0;
  vel_moveB_joint2.shoulder = 0.0;
  vel_moveB_joint2.elbow = -2;
  vel_moveB_joint2.wrist1 = 0.0;
  vel_moveB_joint2.wrist2 = 0.0;
  vel_moveB_joint2.wrist3 = 0.0;

  vel_move_joint1.base = 0;
  vel_move_joint1.shoulder = 0.2;
  vel_move_joint1.elbow = 0.0;
  vel_move_joint1.wrist1 = 0.0;
  vel_move_joint1.wrist2 = 0.0;
  vel_move_joint1.wrist3 = 0.0;

  vel_move_joint2.base = 0;
  vel_move_joint2.shoulder = 0.0;
  vel_move_joint2.elbow = -0.2;
  vel_move_joint2.wrist1 = 0.0;
  vel_move_joint2.wrist2 = 0.0;
  vel_move_joint2.wrist3 = 0.0;

  vel_stop.base = 0;
  vel_stop.shoulder = 0.0;
  vel_stop.elbow = 0;
  vel_stop.wrist1 = 0;
  vel_stop.wrist2 = 0;
  vel_stop.wrist3 = 0;

  ROS_INFO("Start.");

  chatter_pub.publish(vel_moveF_joint1);
  usleep(800000);
  chatter_pub.publish(vel_moveB_joint1);
  usleep(800000);
  chatter_pub.publish(vel_moveF_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveB_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveF_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveB_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveF_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveB_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveF_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveB_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveF_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveB_joint1);
  usleep(600000);
  chatter_pub.publish(vel_moveF_joint1);
  usleep(600000);
  chatter_pub.publish(vel_stop);
  sleep(1);

  chatter_pub.publish(vel_moveF_joint2);
  sleep(1);
  chatter_pub.publish(vel_moveB_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveF_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveB_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveF_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveB_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveF_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveB_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveF_joint2);
  usleep(800000);
  chatter_pub.publish(vel_moveB_joint2);
  usleep(800000);
  chatter_pub.publish(vel_stop);
  sleep(1);

//  chatter_pub.publish(vel_move_joint1);
//  sleep(5);
//  chatter_pub.publish(vel_move_joint2);
//  sleep(5);
//  chatter_pub.publish(vel_stop);
//  sleep(2);

  ROS_INFO("End.");

  return 0;
}
