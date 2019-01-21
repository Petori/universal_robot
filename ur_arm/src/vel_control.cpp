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
#include <unistd.h>   // for function usleep(microseconds)

// Global Variables
std::ofstream fout("data/test.txt");
std::vector<double> curPos;
std::vector<double> curVel;
std::vector<double> curEff;
geometry_msgs::Twist velNew;
geometry_msgs::Twist velInitial;
geometry_msgs::Twist velStop;
bool collisionHappen = false;
bool rule = false;// the collision judging rule.
ur_arm::Joints torque;

// Function definition
void recordToTxt(sensor_msgs::JointState curState);// The callback func for subscriber"recorder"
void velCompute(ur_arm::Joints exTorque);// Judge if collision happens and compute newVel
void setVelStop();
void setVelInitial();
double reZero(double x);

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/ur_arm/cmd_tool_vel", 1);
  ros::Subscriber recorder = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, recordToTxt);// Subscribing the joint_states and record them.
  ros::Subscriber collisionCheck = n.subscribe<ur_arm::Joints>("/external_torque", 1, velCompute);
  usleep(400000);//Leave 0.4s for building the subscriber

  setVelInitial();
  setVelStop();
  velNew = velInitial;
  vel_pub.publish(velInitial);
  sleep(1);

  while(ros::ok())
  {
      // Rule definition
      rule = ((torque.shoulder>7.5) ||(torque.elbow>7.5));
      // end...
      if (rule)
      {
          collisionHappen = true;
      }
      else
      {
          collisionHappen = false;
      }
      //ROS_INFO("Collision state is [%d].", collisionHappen);
      if(collisionHappen)
          {
          geometry_msgs::Twist oldVel;
          geometry_msgs::Vector3 linear;
          geometry_msgs::Vector3 angular;

          oldVel = velNew;
          linear = oldVel.linear;
          angular = oldVel.angular;

          linear.x = -linear.x;
          linear.y = -linear.y;
          linear.z = -linear.z;
          angular.x = -angular.x;
          angular.y = -angular.y;
          angular.z = -angular.z;

          velNew.linear = linear;
          velNew.angular = angular;
          vel_pub.publish(velNew);
          sleep(1);
          }
  }
  fout.close();
  return 0;
}

void setVelInitial()
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
    velInitial.linear = linear;
    velInitial.angular = angular;
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

void velCompute(ur_arm::Joints exTorque)
{
    torque = exTorque;
}

void recordToTxt(sensor_msgs::JointState curState)
{
    // After test, I know that this function is called 125 times per second in real robot connection.
    std::vector<std::string> curName;

    curName = curState.name;
    curPos = curState.position;
    curVel = curState.velocity;
    curEff = curState.effort;

    // I dont record these data because I dont use them.
    fout<<"header:"<<std::endl;
    fout<<"  seq: "<<"000000"<<std::endl;
    fout<<"  stamp:"<<std::endl;
    fout<<"    secs: "<<"000000"<<std::endl;
    fout<<"    nsecs: "<<"000000"<<std::endl;
    fout<<"  frame_id: \'\'"<<std::endl;

    // write the name;
    fout <<"name: [";
    for(int i=0; i<(curName.size()-1); ++i)
    {
      fout <<'\'';
      fout << curName[i] << "\', ";
    }
    fout<<'\''<<curName[curName.size()-1]<<'\''<<']'<<std::endl;

    // write the position;
    fout <<"position: [";
    for(int i=0; i<(curPos.size()-1); ++i)
    {
      fout << curPos[i] << ", ";
    }
    fout<<curPos[curPos.size()-1]<<']'<<std::endl;

    // write the velocity
    fout <<"velocity: [";
    for(int i=0; i<(curVel.size()-1); ++i)
    {
      fout << curVel[i] <<", ";
    }
    fout<<curVel[curVel.size()-1]<<']'<<std::endl;

    // write the effort
    fout <<"effort: [";
    for(int i=0; i<(curEff.size()-1); ++i)
    {
      fout << curEff[i] << ", ";
    }
    fout<<curEff[curEff.size()-1]<<']'<<std::endl;
    fout<<"---"<<std::endl;
}
