// -----------------------------------------------------------------------------------------------
// version 1.0 created by petori on 2018/11/14
// version 2.0 on 2019/3/26
// This file is used to get six-dimensional force data from ATI Axia80.
// The bias is set.
// The gravity compensation is done.
// This file is relied on the raw data from netft_utils package.
// -----------------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include "ur_arm/Joints.h"
#include "ur_arm/my_func.h"
#include "ur_arm/PoseMatrix.h"
#include <unistd.h>   // for function usleep(microseconds)

// Global Variables
static std::vector<double> curPos;
double m = 0.16; // the weight of sensor and tool / or 0.28
double g = 9.793;
double d = 0.05; // the distance between sensor's output and robot's flange
static geometry_msgs::WrenchStamped rawForce;
static geometry_msgs::WrenchStamped biasW;
static uint32_t seqseq = 1;


// Function definition
void getRawData(geometry_msgs::WrenchStamped awrench);
void getCurRobotState(sensor_msgs::JointState curState);
void biasDealtData(geometry_msgs::WrenchStamped biasWrench);
geometry_msgs::WrenchStamped cacDealtData(geometry_msgs::Vector3 force, std::vector<double> pos);
geometry_msgs::WrenchStamped wrenchSubstractAB(geometry_msgs::WrenchStamped A, geometry_msgs::WrenchStamped B);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getSensorData");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  //spinner.start();
  ros::Rate r(250);

  ros::Subscriber sub1 = n.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1,getRawData);
  ros::Subscriber sub2 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, getCurRobotState);
  ros::Subscriber sub3 = n.subscribe<geometry_msgs::WrenchStamped>("/bias_dealt_data", 1,biasDealtData);
  ros::Publisher pub1 = n.advertise<geometry_msgs::WrenchStamped>("/dealt_data", 1);
  usleep(500000);//Leave 0.5s for building the publisher and subscriber

  ros::spinOnce();
  r.sleep();
  biasW = rawForce;
  ROS_INFO("Please waiting for one second ....");
  sleep(1.0);
  ROS_INFO("Ok, this program is running normally.");

  geometry_msgs::WrenchStamped *data = new geometry_msgs::WrenchStamped;
  while(ros::ok())
  {
      ros::spinOnce();
      r.sleep();
      geometry_msgs::WrenchStamped temp;
      temp = rawForce;
      *data = wrenchSubstractAB(temp, biasW);
      *data = cacDealtData((*data).wrench.force, curPos);
      pub1.publish(*data);
      //usleep(8000);
  }

  delete data;
  return 0;
}

void getCurRobotState(sensor_msgs::JointState curState)
{
    curPos = curState.position;
}

void getRawData(geometry_msgs::WrenchStamped awrench)
{
    rawForce = awrench;
}

void biasDealtData(geometry_msgs::WrenchStamped biasWrench)
{
    biasW =   biasWrench;
}

geometry_msgs::WrenchStamped cacDealtData(geometry_msgs::Vector3 force, std::vector<double> pos)
{
    geometry_msgs::WrenchStamped ws;
    std_msgs::Header header; // Not assigned now.
    geometry_msgs::Wrench wrench;

    ur_arm::PoseMatrix pose;
    Eigen::MatrixXf poseRot(3,3);
    Eigen::MatrixXf rawF(3,1);
    Eigen::MatrixXf graInflu(3,1);
    Eigen::MatrixXf cacF(3,1);
    geometry_msgs::Vector3 dealtForce;

    pose = fKine(pos);
    poseRot(0,0) = pose.n[0];
    poseRot(1,0) = pose.n[1];
    poseRot(2,0) = pose.n[2];
    poseRot(0,1) = pose.o[0];
    poseRot(1,1) = pose.o[1];
    poseRot(2,1) = pose.o[2];
    poseRot(0,2) = pose.a[0];
    poseRot(1,2) = pose.a[1];
    poseRot(2,2) = pose.a[2];

    rawF(0,0) = force.x;
    rawF(1,0) = force.y;
    rawF(2,0) = force.z;

    graInflu(0,0) = 0;
    graInflu(1,0) = 0;
    graInflu(2,0) = 0;//m*g;

    // transform the force into base frame
    // and eliminate the bias and gravity
    cacF = poseRot*rawF + graInflu;// - bias + poseRot.transpose()*graInflu;

    dealtForce.x = cacF(0,0);
    dealtForce.y = cacF(1,0);
    dealtForce.z = cacF(2,0);

    header.seq = seqseq;
    seqseq ++;
    header.stamp = ros::Time::now();
    header.frame_id = 1;// global frame;

    wrench.force = dealtForce;
    wrench.torque.x = 0;// temporaty
    wrench.torque.y = 0;
    wrench.torque.z = 0;

    ws.header = header;
    ws.wrench = wrench;

    return ws;
}

geometry_msgs::WrenchStamped wrenchSubstractAB(geometry_msgs::WrenchStamped A, geometry_msgs::WrenchStamped B)
{
    geometry_msgs::WrenchStamped C;
    C.wrench.force.x = A.wrench.force.x - B.wrench.force.x;
    C.wrench.force.y = A.wrench.force.y - B.wrench.force.y;
    C.wrench.force.z = A.wrench.force.z - B.wrench.force.z;
    return C;
}
