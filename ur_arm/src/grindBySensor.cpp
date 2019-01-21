#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
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
#include "netft_utils/SetBias.h"
#include <queue>

using std::cout;
using std::endl;

// Global Variables
std::ofstream fout1("data/jointsRecord.txt");
std::vector<double> curPos;
geometry_msgs::Twist velFoward;
geometry_msgs::Twist velBack;
geometry_msgs::Twist velMove;
geometry_msgs::Twist velMoveReverse;
geometry_msgs::Twist velStop;
geometry_msgs::WrenchStamped wrenchBias;
geometry_msgs::WrenchStamped wrenchRaw;
geometry_msgs::WrenchStamped wrenchReal;
bool collisionHappen = false;
bool rule = false;// the collision judging rule.
ur_arm::Joints torque;
double collisionForce = 0;
double collisonThreshold = 3;

// Function definition
void jointStateGet(sensor_msgs::JointState curState);
void exTorGet(geometry_msgs::WrenchStamped awrench);// Judge if collision happens and compute newVel
void setVelFoward();
void setVelBack();
void setVelMove();
void setVelMoveReverse();
void setVelStop();
void recordPointInfo(std::vector<double> pos, geometry_msgs::WrenchStamped awrench);
geometry_msgs::WrenchStamped wrenchSubstract(geometry_msgs::WrenchStamped awrench1, geometry_msgs::WrenchStamped awrench2);
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
  ros::Subscriber collisionCheck = n.subscribe<geometry_msgs::WrenchStamped>("/transformed_world", 1, exTorGet);
  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, jointStateGet);
  usleep(500000);//Leave 0.5s for building the subscribers and publishers

  // for servcie call
  netft_utils::SetBias bias;
  bias.request.toBias = true;
  bias.request.forceMax = 50;
  bias.request.torqueMax = 10;

  setVelFoward();
  setVelBack();
  setVelMove();
  setVelMoveReverse();
  setVelStop();

  bool rule = false;
  int testPointNum = 25;

//  double distanceInterval = 0.04; So the move time is 2s.
  signal(SIGINT, Stop);// deal with the "ctrl + C"

  for(int i=0;i<testPointNum;i++)
  {
      rule = false;
      if(ros::service::call("/bias",bias))
          {
          ROS_INFO("You called the bias.");
      }
      wrenchBias = wrenchRaw;
      vel_pub.publish(velFoward);
      sleep(1);
      double a=0;
      double b=0;
      double c=0;
      while(!rule&&ros::ok())
      {
          //wrenchReal = wrenchSubstract(wrenchRaw, wrenchBias);
          wrenchReal = wrenchRaw;
          a = wrenchReal.wrench.force.x;
          b = wrenchReal.wrench.force.y;
          c = wrenchReal.wrench.force.z;
          collisionForce = sqrt(a*a + b*b + c*c);
          rule = (collisionForce>collisonThreshold);
      }
      recordPointInfo(curPos, wrenchReal);
      ROS_INFO("I got [%d] point.",i+1);
      vel_pub.publish(velBack);
      usleep(500000);
      if (i == testPointNum-1)
          {
          vel_pub.publish(velStop);
          break;
      }
      vel_pub.publish(velMove);
      sleep(1);
      vel_pub.publish(velStop);
      sleep(1);
  }
  sleep(1);
  vel_pub.publish(velMoveReverse);
  sleep(testPointNum-1);
  vel_pub.publish(velStop);
  ROS_INFO("Exploration finished.");
  ROS_INFO("I have generated the jointsRecord.txt file.");
  return 0;
}

void exTorGet(geometry_msgs::WrenchStamped awrench)
{
    wrenchRaw = awrench;
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
    vy = 0.05;
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
    vx = 0.005;
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

void setVelMoveReverse()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = -0.005;
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
    velMoveReverse.linear = linear;
    velMoveReverse.angular = angular;
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

void recordPointInfo(std::vector<double> pos, geometry_msgs::WrenchStamped awrench)
{
    fout1<<ros::Time::now()<<",";
    fout1<<pos[0]<<", ";
    fout1<<pos[1]<<", ";
    fout1<<pos[2]<<", ";
    fout1<<pos[3]<<", ";
    fout1<<pos[4]<<", ";
    fout1<<pos[5]<<",";
    fout1<<awrench.wrench.force.x<<",";
    fout1<<awrench.wrench.force.y<<",";
    fout1<<awrench.wrench.force.z<<",";
    fout1<<awrench.wrench.torque.x<<",";
    fout1<<awrench.wrench.torque.y<<",";
    fout1<<awrench.wrench.torque.z;
    fout1<<std::endl;
}

geometry_msgs::WrenchStamped wrenchSubstract(geometry_msgs::WrenchStamped awrench1, geometry_msgs::WrenchStamped awrench2)
{
    geometry_msgs::WrenchStamped result;

    result.header = awrench1.header;
    result.wrench.force.x = awrench1.wrench.force.x - awrench2.wrench.force.x;
    result.wrench.force.y = awrench1.wrench.force.y - awrench2.wrench.force.y;
    result.wrench.force.z = awrench1.wrench.force.z - awrench2.wrench.force.z;
    result.wrench.torque.x = awrench1.wrench.torque.x - awrench2.wrench.torque.x;
    result.wrench.torque.y = awrench1.wrench.torque.y - awrench2.wrench.torque.y;
    result.wrench.torque.z = awrench1.wrench.torque.z - awrench2.wrench.torque.z;

    return result;
}
