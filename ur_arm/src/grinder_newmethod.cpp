#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"
#include "ur_arm/my_func.h"
#include <unistd.h>   // for function usleep(microseconds)
#include <cstdlib>
#include <signal.h>   // deal with the "ctrl + C"

using std::cout;
using std::endl;

// Global Variables
std::ofstream fout1("data/jointsRecord.txt");
std::vector<double> curPos;
geometry_msgs::Twist velFoward;
geometry_msgs::Twist velBack;
geometry_msgs::Twist velMove;
geometry_msgs::Twist velStop;
bool collisionHappen = false;
bool rule = false;// the collision judging rule.
ur_arm::Joints torque;
double collisionTorque1 = 0.3;
double collisionTorque2 = 0.3;

// Function definition
void jointStateGet(sensor_msgs::JointState curState);
void exTorGet(ur_arm::Joints exTorque);// Judge if collision happens and compute newVel
void setVelFoward();
void setVelBack();
void setVelMove();
void setVelStop();
void recordJoints(std::vector<double> pos);
void Stop(int signo)
{
    printf("oops! You stopped the program!\n");
    _exit(0);
}

// Main
int main(int argc, char **argv)
{
  hi();
  ros::init(argc, argv, "grinder_newmethod");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/ur_arm/cmd_tool_vel", 1);
  ros::Subscriber collisionCheck = n.subscribe<ur_arm::Joints>("/external_torque", 1, exTorGet);
  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, jointStateGet);
  usleep(500000);//Leave 0.5s for building the subscribers and publishers

  setVelFoward();
  setVelBack();
  setVelMove();
  setVelStop();

  bool rule = false;
  int testPointNum = 7;

//  double distanceInterval = 0.04; So the move time is 2s.
  signal(SIGINT, Stop);// deal with the "ctrl + C"

  for(int i=0;i<testPointNum;i++)
  {
      ROS_INFO("Hey.");
      rule = false;
      vel_pub.publish(velFoward);
      while(!rule&&ros::ok())
      {
          rule = ((torque.shoulder>collisionTorque1) || (torque.elbow>collisionTorque2));
      }
      recordJoints(curPos);
      vel_pub.publish(velBack);
      sleep(5);
      vel_pub.publish(velMove);
      sleep(3);
      vel_pub.publish(velStop);
      sleep(1);
  }
  ROS_INFO("I have generated the jointsRecord.txt file.");
  return 0;
}

void exTorGet(ur_arm::Joints exTorque)
{
    torque = exTorque;
}

void setVelFoward()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = -0.005;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velFoward.linear = linear;
    velFoward.angular = angular;
}

void setVelBack()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = 0.005;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velBack.linear = linear;
    velBack.angular = angular;
}

void setVelMove()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0.02;
    vy = 0;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velMove.linear = linear;
    velMove.angular = angular;
}

void setVelStop()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = 0;
    vz = 0;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velStop.linear = linear;
    velStop.angular = angular;
}

void jointStateGet(sensor_msgs::JointState curState)
{
    // After test, I know that this function is called 125 times per second in real robot connection.
    curPos = curState.position;
}

void recordJoints(std::vector<double> pos)
{
    std::vector<double> posTmp;
    // Maybe posTmp can not be assigned like this.
    posTmp = pos;
    fout1<<"[";
    fout1<<posTmp[0]<<", ";
    fout1<<posTmp[1]<<", ";
    fout1<<posTmp[2]<<", ";
    fout1<<posTmp[3]<<", ";
    fout1<<posTmp[4]<<", ";
    fout1<<posTmp[5]<<"]"<<std::endl;
}
