#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"
#include <unistd.h>   // for function usleep(microseconds)

// Global Variables
ur_arm::Joints exTorque;
Eigen::MatrixXf exTorque2(2,1);
Eigen::MatrixXf A(2,1);

// Function definition
void getCurRobotState(sensor_msgs::JointState curState);// The callback func for subscriber"monitor", get cur pos/vel/eff values.
ur_arm::Joints computeExTorque(std::vector<double> curPos, std::vector<double> curVel, std::vector<double> curEff);
void setInitialExTorque();
double reZeroForVel(double x);

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_detect");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  setInitialExTorque();
  ros::Publisher collision_pub = n.advertise<ur_arm::Joints>("/external_torque",1);
  ros::Subscriber monitor = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, getCurRobotState);// Subscribing the joint_states for collision compute.
  usleep(400000);//Leave 0.4s for building the publisher and subscriber

  while(ros::ok())
  {
      collision_pub.publish(exTorque);
      usleep(8000);
  }
  return 0;
}

void setInitialExTorque()
{
    exTorque.base = 0;
    exTorque.shoulder = 0;
    exTorque.elbow = 0;
    exTorque.wrist1 = 0;
    exTorque.wrist2 = 0;
    exTorque.wrist3 = 0;
    exTorque2(0,0) = 0;
    exTorque2(1,0) = 0;
    A(0,0) = 0;
    A(1,0) = 0;
}

void getCurRobotState(sensor_msgs::JointState curState)
{
    std::vector<double> curPos;
    std::vector<double> curVel;
    std::vector<double> curEff;
    curPos = curState.position;
    curVel = curState.velocity;
    curEff = curState.effort;
    exTorque = computeExTorque(curPos, curVel, curEff);
}

ur_arm::Joints computeExTorque(std::vector<double> curPos, std::vector<double> curVel, std::vector<double> curEff)
{
    // toliaezi give the curEff to exTorque
    ur_arm::Joints torque;
//    // the parameter of ur5
//    double m1 = 0.8009;
//    double m2 = 0.5515;
//    double l1_star = 0.1320;
//    double l2_star = 0.2926;
//    double u1_1=0.2973;
//    double u2_1=0.6849;
//    double u1_2=0.5823;
//    double u2_2=0.6754;

//    // the parameter with grinder
    double m1 = 0.4254;
    double m2 = 0.3755;
    double l1_star = 0.5689;
    double l2_star = 0.5818;
    double u1_1=0.5822;
    double u2_1=0.5286;
    double u1_2=0.6702;
    double u2_2=0.6448;
    double l1 = 0.425;
    double g = 9.793;

    double q2,q3,dq2,dq3,r_dq2,r_dq3;

    q2 = curPos[1];
    q3 = curPos[2];

    dq2 = curVel[1];
    dq3 = curVel[2];

    r_dq2 = reZeroForVel(curVel[1]);
    r_dq3 = reZeroForVel(curPos[2]);

    torque.shoulder = fabs(-m1*g*l1_star*cos(q2) - m2*g*l1*cos(q2) - m2*g*l2_star*cos(q2 + q3) + u1_1*dq2 + u2_1*signed(r_dq2));
    torque.elbow = fabs(-m2*g*l2_star*cos(q2+q3) + u1_2*dq3 + u2_2*signed(r_dq3));

    torque.base = fabs(curEff[0]);
    torque.wrist1 = fabs(curEff[3]);
    torque.wrist2 = fabs(curEff[4]);
    torque.wrist3 = fabs(curEff[5]);

    ROS_INFO("External Torque of elbow =  [%lf].",torque.elbow);

    return torque;
}

double reZeroForVel(double x)
{
    // vel noise is 0.02 rad/s.
    if (fabs(x)<0.02)
    {
        x = 0;
    }
    return x;
}
