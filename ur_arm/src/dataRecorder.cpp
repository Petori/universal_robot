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

// Global Variables
std::ofstream fout1("data/jointStates[dR].txt");
std::ofstream fout2("data/externalTorque[dR].txt");
std::ofstream fout3("data/netftData[dR].txt");
std::ofstream fout4("data/rawWorld[dR].txt");
std::ofstream fout5("data/transWorld[dR].txt");
std::ofstream fout6("data/transTool[dR].txt");
std::ofstream fout7("data/dealtData[dR].txt");
std::vector<double> curPos;
std::vector<double> curVel;
std::vector<double> curEff;

// Function definition
void recordJointStateToTxt(sensor_msgs::JointState curState);
void recordExternalTorqueToTxt(ur_arm::Joints joints);
void recordNetftDataToTxt(geometry_msgs::WrenchStamped awrench);
void recordRawWorldDataToTxt(geometry_msgs::WrenchStamped awrench);
void recordTransWorldDataToTxt(geometry_msgs::WrenchStamped awrench);
void recordTransToolDataToTxt(geometry_msgs::WrenchStamped awrench);
void recordDealtDataToTxt(geometry_msgs::WrenchStamped awrench);

// Main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dataRecorder");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, recordJointStateToTxt);// Subscribing the joint_states and record them.
  ros::Subscriber recorder2 = n.subscribe<ur_arm::Joints>("/external_torque", 1, recordExternalTorqueToTxt);
  ros::Subscriber recorder3 = n.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, recordNetftDataToTxt);
  ros::Subscriber recorder4 = n.subscribe<geometry_msgs::WrenchStamped>("/raw_world", 1, recordRawWorldDataToTxt);
  ros::Subscriber recorder5 = n.subscribe<geometry_msgs::WrenchStamped>("/transformed_world", 1, recordTransWorldDataToTxt);
  ros::Subscriber recorder6 = n.subscribe<geometry_msgs::WrenchStamped>("/transformed_tool", 1, recordTransToolDataToTxt);
  ros::Subscriber recorder7 = n.subscribe<geometry_msgs::WrenchStamped>("/dealt_data", 1, recordDealtDataToTxt);

  usleep(1000000);//Leave 0.5s for building the subscribers and publishers

  while(ros::ok()){};
  fout1.close();
  fout2.close();
  fout3.close();

  return 0;
}

void recordExternalTorqueToTxt(ur_arm::Joints joints)
{
    ur_arm::Joints curJoints;
    curJoints = joints;
    fout2<<"[";
    fout2<<ros::Time::now()<<",";
    fout2<<curJoints.base<<", ";
    fout2<<curJoints.shoulder<<", ";
    fout2<<curJoints.elbow<<", ";
    fout2<<curJoints.wrist1<<", ";
    fout2<<curJoints.wrist2<<", ";
    fout2<<curJoints.wrist3<<"]"<<std::endl;
}

void recordJointStateToTxt(sensor_msgs::JointState curState)
{
    // After test, I know that this function is called 125 times per second in real robot connection.
    std::vector<std::string> curName;

    curName = curState.name;
    curPos = curState.position;
    curVel = curState.velocity;
    curEff = curState.effort;

    // I dont record these data because I dont use them.
    fout1<<"header:"<<std::endl;
    fout1<<"  seq: "<<"000000"<<std::endl;
    fout1<<"  stamp:"<<std::endl;
    fout1<<"    secs: "<<ros::Time::now()<<std::endl;
    fout1<<"    nsecs: "<<"000000"<<std::endl;
    fout1<<"  frame_id: \'\'"<<std::endl;

    // write the name;
    fout1 <<"name: [";
    for(int i=0; i<(curName.size()-1); ++i)
    {
      fout1 <<'\'';
      fout1 << curName[i] << "\', ";
    }
    fout1<<'\''<<curName[curName.size()-1]<<'\''<<']'<<std::endl;

    // write the position;
    fout1 <<"position: [";
    for(int i=0; i<(curPos.size()-1); ++i)
    {
      fout1 << curPos[i] << ", ";
    }
    fout1<<curPos[curPos.size()-1]<<']'<<std::endl;

    // write the velocity
    fout1 <<"velocity: [";
    for(int i=0; i<(curVel.size()-1); ++i)
    {
      fout1 << curVel[i] <<", ";
    }
    fout1<<curVel[curVel.size()-1]<<']'<<std::endl;

    // write the effort
    fout1 <<"effort: [";
    for(int i=0; i<(curEff.size()-1); ++i)
    {
      fout1 << curEff[i] << ", ";
    }
    fout1<<curEff[curEff.size()-1]<<']'<<std::endl;
    fout1<<"---"<<std::endl;
}

void recordNetftDataToTxt(geometry_msgs::WrenchStamped awrench)
{
    geometry_msgs::WrenchStamped curWrench;
    geometry_msgs::Vector3 temp1;
    geometry_msgs::Vector3 temp2;

    curWrench = awrench;
    temp1 = curWrench.wrench.force;
    temp2 = curWrench.wrench.torque;

    fout3<<"[";
    fout3<<ros::Time::now()<<",";
    fout3<<temp1.x<<", ";
    fout3<<temp1.y<<", ";
    fout3<<temp1.z<<", ";
    fout3<<temp2.x<<", ";
    fout3<<temp2.y<<", ";
    fout3<<temp2.z<<"]"<<std::endl;
}

void recordRawWorldDataToTxt(geometry_msgs::WrenchStamped awrench)
{
    geometry_msgs::WrenchStamped curWrench;
    geometry_msgs::Vector3 temp1;
    geometry_msgs::Vector3 temp2;

    curWrench = awrench;
    temp1 = curWrench.wrench.force;
    temp2 = curWrench.wrench.torque;

    fout4<<"[";
    fout4<<ros::Time::now()<<",";
    fout4<<temp1.x<<", ";
    fout4<<temp1.y<<", ";
    fout4<<temp1.z<<", ";
    fout4<<temp2.x<<", ";
    fout4<<temp2.y<<", ";
    fout4<<temp2.z<<"]"<<std::endl;
}

void recordTransWorldDataToTxt(geometry_msgs::WrenchStamped awrench)
{
    geometry_msgs::WrenchStamped curWrench;
    geometry_msgs::Vector3 temp1;
    geometry_msgs::Vector3 temp2;

    curWrench = awrench;
    temp1 = curWrench.wrench.force;
    temp2 = curWrench.wrench.torque;

    fout5<<"[";
    fout5<<ros::Time::now()<<",";
    fout5<<temp1.x<<", ";
    fout5<<temp1.y<<", ";
    fout5<<temp1.z<<", ";
    fout5<<temp2.x<<", ";
    fout5<<temp2.y<<", ";
    fout5<<temp2.z<<"]"<<std::endl;
}

void recordTransToolDataToTxt(geometry_msgs::WrenchStamped awrench)
{
    geometry_msgs::WrenchStamped curWrench;
    geometry_msgs::Vector3 temp1;
    geometry_msgs::Vector3 temp2;

    curWrench = awrench;
    temp1 = curWrench.wrench.force;
    temp2 = curWrench.wrench.torque;

    fout6<<"[";
    fout6<<ros::Time::now()<<",";
    fout6<<temp1.x<<", ";
    fout6<<temp1.y<<", ";
    fout6<<temp1.z<<", ";
    fout6<<temp2.x<<", ";
    fout6<<temp2.y<<", ";
    fout6<<temp2.z<<"]"<<std::endl;
}

void recordDealtDataToTxt(geometry_msgs::WrenchStamped awrench)
{
    geometry_msgs::WrenchStamped curWrench;
    geometry_msgs::Vector3 temp1;
    geometry_msgs::Vector3 temp2;

    curWrench = awrench;
    temp1 = curWrench.wrench.force;
    temp2 = curWrench.wrench.torque;

    fout7<<"[";
    fout7<<ros::Time::now()<<",";
    fout7<<temp1.x<<", ";
    fout7<<temp1.y<<", ";
    fout7<<temp1.z<<", ";
    fout7<<temp2.x<<", ";
    fout7<<temp2.y<<", ";
    fout7<<temp2.z<<"]"<<std::endl;
}
