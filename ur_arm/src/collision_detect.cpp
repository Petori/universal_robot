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
#include "ur_arm/my_func.h"
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
    // the column vector definition
    Eigen::MatrixXf jointTorque2(2,1);
    Eigen::MatrixXf vel2(2,1);
    Eigen::MatrixXf pos2(2,1);
    Eigen::MatrixXf deltaA(2,1);
    Eigen::MatrixXf torqueFric(2,1);
    Eigen::MatrixXf Cq(2,2);
    Eigen::MatrixXf Gq(2,1);
    Eigen::MatrixXf Mq(2,2);
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
    double K = 10;
    double K2 = 1;
    double dt = 0.008;

    pos2(0,0) = curPos[1];
    pos2(1,0) = curPos[2];
    vel2(0,0) = curVel[1];
    vel2(1,0) = curVel[2];
    jointTorque2(0,0) = curEff[1];
    jointTorque2(1,0) = curEff[2];

    Mq(0,0) = m1*l1_star*l1_star + m2*(l1*l1+l1_star*l1_star+2*l1*l1_star*cos(pos2(1,0)));
    Mq(0,1) = m2*(l2_star*l2_star+l1*l1_star*cos(pos2(1,0)));
    Mq(1,0) = m2*(l2_star*l2_star+l1*l1_star*cos(pos2(1,0)));
    Mq(1,1) = m2*l2_star*l2_star;

    Cq(0,0) = -m2*l1*l2_star*sin(pos2(1,0))*vel2(1,0);
    Cq(0,1) = -m2*l1*l2_star*sin(pos2(1,0))*(vel2(0,0)+vel2(1,0));
    Cq(1,0) = -m2*l1*l2_star*sin(pos2(1,0))*(-vel2(0,0));
    Cq(1,1) = 0;

    Gq(0,0) = -(m1*l1_star+m2*l1)*g*cos(pos2(0,0)) - m2*g*l2_star*cos(pos2(0,0)+pos2(1,0));
    Gq(1,0) = -m2*g*l2_star*cos(pos2(0,0)+pos2(1,0));

    deltaA = (jointTorque2 + Cq.transpose()*vel2 - Gq - exTorque2) * dt;
    A = A + deltaA;
    exTorque2 = K*(A - Mq*vel2);

    torqueFric(0,0) = u1_1*curVel[1] + u2_1*signsign(curVel[1]);
    torqueFric(1,0) = u1_2*curVel[2] + u2_2*signsign(curVel[2]);

    torque.base = fabs(K2*curEff[0]);
    torque.shoulder = fabs(K2*(exTorque2(0,0) - torqueFric(0,0)));
    torque.elbow = fabs(K2*(exTorque2(1,0) - torqueFric(1,0)));
    torque.wrist1 = fabs(K2*curEff[3]);
    torque.wrist2 = fabs(K2*curEff[4]);
    torque.wrist3 = fabs(K2*curEff[5]);
    ROS_INFO("External Torque of elbow =  [%lf].",torque.elbow);

    return torque;
}
