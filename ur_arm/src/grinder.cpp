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

using std::cout;
using std::endl;

// Global Variables
std::ofstream fout1("data/jointStates.txt");
std::ofstream fout2("data/externalTorque.txt");
std::ofstream fout3("catkin_ws/src/universal_robot/ur_modern_driver/grinderWithDetect.py");
std::vector<double> curPos;
std::vector<double> curVel;
std::vector<double> curEff;
geometry_msgs::Twist velFoward;
geometry_msgs::Twist velBack;
geometry_msgs::Twist velToPointMid;
geometry_msgs::Twist velToPointEnd;
geometry_msgs::Twist velToPointEndInv;
geometry_msgs::Twist velStop;
bool collisionHappen = false;
bool rule = false;// the collision judging rule.
ur_arm::Joints torque;
double collisionTorque1 = 1;
double collisionTorque2 = 0.9;

// Function definition
void recordJointStateToTxt(sensor_msgs::JointState curState);
void recordExternalTorqueToTxt(ur_arm::Joints joints);
void velCompute(ur_arm::Joints exTorque);// Judge if collision happens and compute newVel
void setVelFoward();
void setVelBack();
void setVelToPointMid();
void setVelToPointEnd();
void setVelToPointEndInv();
void setVelStop();
geometry_msgs::Twist velGet(geometry_msgs::Twist pose1, geometry_msgs::Twist pose2);
void pycodeGenerate(std::vector<double> Point1, std::vector<double> Point2);// Generate the grinder py file

// Main
int main(int argc, char **argv)
{
  hi();
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/ur_arm/cmd_tool_vel", 1);
  ros::Subscriber collisionCheck = n.subscribe<ur_arm::Joints>("/external_torque", 1, velCompute);
  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, recordJointStateToTxt);// Subscribing the joint_states and record them.
  ros::Subscriber recorder2 = n.subscribe<ur_arm::Joints>("/external_torque", 1, recordExternalTorqueToTxt);

  usleep(500000);//Leave 0.5s for building the subscribers and publishers

  setVelFoward();
  setVelBack();
  setVelToPointMid();
  setVelToPointEnd();
  setVelToPointEndInv();
  setVelStop();

  bool rule1 = false;
//  bool rule2 = false;
  bool rule3 = false;
  std::vector<double> startPoint;
  std::vector<double> endPoint;
//  std::vector<double> endPoint;

  vel_pub.publish(velFoward);
  sleep(1);

  // Get 3 key position by collision detect.
  while(!rule1)
  {
      rule1 = ((torque.shoulder>collisionTorque1) || (torque.elbow>collisionTorque2));
  }
  startPoint = curPos;// start position
  vel_pub.publish(velBack);
  sleep(5);
//  vel_pub.publish(velToPointMid);
//  sleep(5);

//  vel_pub.publish(velFoward);
//  while(!rule2)
//  {
//      rule2 = ((torque.shoulder>collisionTorque) || (torque.elbow>collisionTorque));
//  }
//  midPoint = curPos;// mid position
//  vel_pub.publish(velBack);
//  sleep(5);
  vel_pub.publish(velToPointEnd);
  sleep(3);

  vel_pub.publish(velFoward);
  sleep(1);
  while(!rule3)
  {
      rule3 = ((torque.shoulder>collisionTorque1) || (torque.elbow>collisionTorque2));
  }
  endPoint = curPos;// end position
  vel_pub.publish(velBack);
  sleep(5);
  vel_pub.publish(velToPointEndInv);
  sleep(3);
  vel_pub.publish(velStop);
  sleep(1);

//  startPoint.push_back(1);
//  startPoint.push_back(2);
//  startPoint.push_back(3);
//  startPoint.push_back(1);
//  startPoint.push_back(2);
//  startPoint.push_back(3);

//  endPoint.push_back(1.1);
//  endPoint.push_back(2.1);
//  endPoint.push_back(3.1);
//  endPoint.push_back(1.1);
//  endPoint.push_back(2.1);
//  endPoint.push_back(3.1);

  pycodeGenerate(startPoint,endPoint);
  fout3.close();
  ROS_INFO("I have generated the grinderWithDetect.py file.");
 // system("rosrun ur_modern_driver grinderWithDetect.py");
// Should I have to sleep some time here?

  while(ros::ok()){};
  fout1.close();
  fout2.close();

  return 0;
}

void pycodeGenerate(std::vector<double> Point1, std::vector<double> Point2)
{
    ur_arm::PoseMatrix startPose;
    ur_arm::PoseMatrix endPose;
    ur_arm::AllAng allangForJudge;
    std::vector<double> P1;
    std::vector<double> P2;
    int solutionNum = 0;
    int Num = 0;

    for(int i = 0;i<6;i++)
        {
        P1.push_back(thetaMod(Point1[i]));
        P2.push_back(thetaMod(Point2[i]));
    }

    startPose = fKine(P1);
    endPose = fKine(P2);

    //% 判断始末构型位于哪组解系
    allangForJudge = invKine(startPose);

    // fabs(*)<0.01 is the best , do not change it.
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang1[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang1[6]==1)){solutionNum = 1;}
    }
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang2[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang2[6]==1)){solutionNum = 2;}
    }
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang3[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang3[6]==1)){solutionNum = 3;}
    }
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang4[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang4[6]==1)){solutionNum = 4;}
    }
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang5[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang5[6]==1)){solutionNum = 5;}
    }
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang6[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang6[6]==1)){solutionNum = 6;}
    }
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang7[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang7[6]==1)){solutionNum = 7;}
    }
    for(int i=0;i<6;i++)
    {
        if (fabs(allangForJudge.ang8[i] - P1[i])<0.001){Num += 10;}
        else {Num = 0;break;}
        if((Num == 60)&&(allangForJudge.ang8[6]==1)){solutionNum = 8;}
    }

    if(solutionNum == 0)
    {
        std::cout<<"解系判断未成功！"<<std::endl;
    }

    //% 笛卡尔空间插值并生成关节角
    int interNum = 100;// 插值个数
    double moveTime = 5;// 打磨时长
    double startTime = 5;
    double deltaTime;
    double tTime[interNum+1];
    double AngAng[interNum+1][6];

    ur_arm::PoseMatrix deltaPose;

    double newTime;

    deltaTime = moveTime/double(interNum);

    for(int i=0;i<3;i++) deltaPose.p[i] = (endPose.p[i] - startPose.p[i])/double(interNum);
    double deltax;
    double deltay;
    double deltaz;
    deltax = deltaPose.p[0];
    deltay = deltaPose.p[1];
    deltaz = deltaPose.p[2];

    newTime = startTime - deltaTime;

    for (int i=0;i<(interNum+1);i++)
    {
        ur_arm::AllAng midAllAng;
        ur_arm::PoseMatrix newPose;
        double midAng_mark[7];

        newPose.n = startPose.n;
        newPose.o = startPose.o;
        newPose.a = startPose.a;

        newPose.p[0] = startPose.p[0] + i*deltax;
        newPose.p[1] = startPose.p[1] + i*deltay + 0.0015 + double(interNum - i)/double(interNum)*0.001;
        newPose.p[2] = startPose.p[2] + i*deltaz;

        midAllAng = invKine(newPose);

        if(solutionNum == 1){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang1[i];}}
        if(solutionNum == 2){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang2[i];}}
        if(solutionNum == 3){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang3[i];}}
        if(solutionNum == 4){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang4[i];}}
        if(solutionNum == 5){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang5[i];}}
        if(solutionNum == 6){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang6[i];}}
        if(solutionNum == 7){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang7[i];}}
        if(solutionNum == 8){ for(int i =0;i<6;i++){ midAng_mark[i] = midAllAng.ang8[i];}}

   // cout<<midAllAng.ang1[0]<<","<<midAllAng.ang2[1]<<","<<midAllAng.ang3[2]<<","<<midAllAng.ang4[3]<<endl;
        for(int k=0;k<6;k++)
        {AngAng[i][k] = midAng_mark[k];}

        newTime = newTime + deltaTime;
        tTime[i] = newTime;
    }


    fout3<<"#!/usr/bin/env python\n";
    fout3<<"import time\n";
    fout3<<"import roslib; roslib.load_manifest(\'ur_modern_driver\')\n";
    fout3<<"import rospy\n";
    fout3<<"import actionlib\n";
    fout3<<"import sys\n";
    fout3<<"import subprocess\n";
    fout3<<"import os\n";
    fout3<<"from control_msgs.msg import *\n";
    fout3<<"from trajectory_msgs.msg import *\n";
    fout3<<"from sensor_msgs.msg import JointState\n";
    fout3<<"from math import pi\n\n";
    fout3<<"JOINT_NAMES = [\'shoulder_pan_joint\', \'shoulder_lift_joint\', \'elbow_joint\', \'wrist_1_joint\', \'wrist_2_joint\', \'wrist_3_joint\']\n\n";
    for(int i=0;i<(interNum+1);i++)
    {
        fout3<<"Q";
        fout3<<(i+1);
        fout3<<" = [";
        fout3<<AngAng[i][0]<<", ";
        fout3<<AngAng[i][1]<<", ";
        fout3<<AngAng[i][2]<<", ";
        fout3<<AngAng[i][3]<<", ";
        fout3<<AngAng[i][4]<<", ";
        fout3<<AngAng[i][5]<<"]\n";
    }
    fout3<<"\n\nclient = None\n";
    fout3<<"def move():\n";
    fout3<<"    global joints_pos\n";
    fout3<<"    g = FollowJointTrajectoryGoal()\n";
    fout3<<"    g.trajectory = JointTrajectory()\n";
    fout3<<"    g.trajectory.joint_names = JOINT_NAMES\n";
    fout3<<"    try:\n";
    fout3<<"        joint_states = rospy.wait_for_message(\"joint_states\", JointState)\n";
    fout3<<"        joints_pos = joint_states.position\n";
    fout3<<"        g.trajectory.points = [\n";
    fout3<<"            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),\n";

    for(int i=0;i<(interNum);i++)
    {
    fout3<<"            JointTrajectoryPoint(positions=Q";
    fout3<<(i+1)<<", velocities=[0]*6, time_from_start=rospy.Duration(";
    fout3<<tTime[i]<<")),\n";
    }

    fout3<<"            JointTrajectoryPoint(positions=Q";
    fout3<<(interNum+1)<<", velocities=[0]*6, time_from_start=rospy.Duration(";
    fout3<<tTime[interNum]<<"))]\n";
    fout3<<"        client.send_goal(g)\n";
    fout3<<"        client.wait_for_result()\n";
    fout3<<"    except KeyboardInterrupt:\n";
    fout3<<"        client.cancel_goal()\n";
    fout3<<"        raise\n";
    fout3<<"    except:\n";
    fout3<<"        raise\n\n";
    fout3<<"def main():\n";
    fout3<<"    global client\n";
    fout3<<"    try:\n";
    fout3<<"        rospy.init_node(\"grinder_move\", anonymous=True, disable_signals=True)\n";
    fout3<<"        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)\n";
    fout3<<"        print \"Waiting for server...\"\n";
    fout3<<"        client.wait_for_server()\n";
    fout3<<"        print \"Connected to server\"\n";
    fout3<<"        child1 = subprocess.Popen(\'rostopic echo /joint_states >1.txt\',shell=True)\n";
    fout3<<"        move()\n";
    fout3<<"        print \"Trajectory finished\"\n";
    fout3<<"        time.sleep(0.5)\n";
    fout3<<"        child2 = subprocess.Popen(\'cp 1.txt data/polish.txt\',shell=True)\n";
    fout3<<"        time.sleep(0.5)\n";
    fout3<<"        child3 = subprocess.Popen(\'rm 1.txt\',shell=True)\n";
    fout3<<"        time.sleep(0.5)\n";
    fout3<<"        if True:\n";
    fout3<<"            child1.kill()\n";
    fout3<<"            child2.kill()\n";
    fout3<<"            child3.kill()\n";
    fout3<<"    except KeyboardInterrupt:\n";
    fout3<<"        rospy.signal_shutdown(\"KeyboardInterrupt\")\n";
    fout3<<"        raise\n\n";
    fout3<<"if __name__ == \'__main__\': main()\n";
}

geometry_msgs::Twist velGet(geometry_msgs::Twist pose1,geometry_msgs::Twist pose2)
{
    geometry_msgs::Vector3 linear1,linear2,linearVel;
    geometry_msgs::Vector3 angular;
    geometry_msgs::Twist vel;
    double goTime = 5;

    linear1 = pose1.linear;
    linear2 = pose2.linear;

    linearVel.x = (linear2.x - linear1.x)/goTime;
    linearVel.y = (linear2.y - linear1.y)/goTime;
    linearVel.z = (linear2.z - linear1.z)/goTime;
    angular.x = 0;
    angular.y = 0;
    angular.z = 0;

    vel.linear = linearVel;
    vel.angular = angular;

    return vel;
}

void velCompute(ur_arm::Joints exTorque)
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

void setVelToPointMid()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = 0;
    vy = 0;
    vz = -0.005;
    wx = 0;
    wy = 0;
    wz = 0;
    linear.x = vx;
    linear.y = vy;
    linear.z = vz;
    angular.x = wx;
    angular.y = wy;
    angular.z = wz;
    velToPointMid.linear = linear;
    velToPointMid.angular = angular;
}

void setVelToPointEnd()
{
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;
    double vx,vy,vz;
    double wx,wy,wz;
    vx = -0.02;
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
    velToPointEnd.linear = linear;
    velToPointEnd.angular = angular;
}

void setVelToPointEndInv()
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
    velToPointEndInv.linear = linear;
    velToPointEndInv.angular = angular;
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

void recordExternalTorqueToTxt(ur_arm::Joints joints)
{
    ur_arm::Joints curJoints;
    curJoints = joints;
    fout2<<"[";
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
    fout1<<"    secs: "<<"000000"<<std::endl;
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
