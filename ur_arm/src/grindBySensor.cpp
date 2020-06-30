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
using std::queue;

// Global Variables
std::ofstream fout1("data/jointsRecord.txt");
std::ofstream fout2("data/virtualStiffness.txt");
static sensor_msgs::JointState jointState;
static geometry_msgs::Twist velFoward;
static geometry_msgs::Twist velBack;
static geometry_msgs::Twist velMove;
static geometry_msgs::Twist velMoveReverse;
static geometry_msgs::Twist velStop;
static geometry_msgs::WrenchStamped wrenchBias;
static geometry_msgs::WrenchStamped wrenchRaw;
static geometry_msgs::WrenchStamped wrenchNow;
bool collisionHappen = false;
bool rule = false;// the collision judging rule.
static ur_arm::Joints torque;
double collisionForce = 0;
double contactForce = 1;
double collisonThreshold = 3;
double stepCoefficient = 1;
//int testPointNum = 10;
double exploreDistance = 0.2;
double nowDistance = 0;
double moveSpeed = -0.05;
double moveReverseSpeed = -0.05;
double exploreSpeed = 0.01;
double backSpeed = 0.07;
int explorePointNum = 0;
double minStiffness = 500;
double maxStiffness = 1500;
double minMoveTime = 0.3; // when the exploreSpeed = 0.05
double maxMoveTime = 0.7;
double rigidMoveTime = 0.5;
double coef_a; // the coefficients for step calculation
double coef_b;

struct datapack
{
    sensor_msgs::JointState robotState;
    geometry_msgs::WrenchStamped sensorData;
    double forceAll;
};

// Function definition
void jointStateGet(sensor_msgs::JointState curState);
void exTorGet(geometry_msgs::WrenchStamped awrench);// Judge if collision happens and calculate newVel
void biasGET(geometry_msgs::WrenchStamped awrench);// Get bias
void setVelFoward();
void setVelBack();
void setVelMove();
void setVelMoveReverse();
void setVelStop();
void recordPointInfo(queue<datapack> qq);
geometry_msgs::WrenchStamped wrenchSubstract(geometry_msgs::WrenchStamped awrench1, geometry_msgs::WrenchStamped awrench2);
datapack packAssign(sensor_msgs::JointState js, geometry_msgs::WrenchStamped ws, double db);
double calculateStep(queue<datapack> qq, double cc);
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
  ros::Publisher bias_pub = n.advertise<geometry_msgs::WrenchStamped>("/bias_dealt_data",1);
  ros::Subscriber collisionCheck = n.subscribe<geometry_msgs::WrenchStamped>("/dealt_data", 1, exTorGet);
  ros::Subscriber sub1 = n.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, biasGET);
  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, jointStateGet);
  usleep(500000);//Leave 0.5s for building the subscribers and publishers

  coef_a = (minMoveTime - maxMoveTime)/(maxStiffness - minStiffness);
  coef_b = minMoveTime - coef_a*maxStiffness;

  // for servcie call bias ---- bias the topic /transformed_word
  netft_utils::SetBias bias;
  bias.request.toBias = true;
  bias.request.forceMax = 50;
  bias.request.torqueMax = 10;

  // for program end judging
  ur_arm::PoseMatrix startpose;
  startpose = fKine(jointState.position);

  setVelFoward();
  setVelBack();
  setVelMove();
  setVelMoveReverse();
  setVelStop();

  bool rule = false;
  queue<datapack> queue20;
  datapack tempPack;
  sensor_msgs::JointState nowState;
  geometry_msgs::WrenchStamped nowWrench;

//  double distanceInterval = 0.04; So the move time is 2s.
  signal(SIGINT, Stop);// deal with the "ctrl + C"

  while(nowDistance<exploreDistance)
  {
      explorePointNum++;
      queue<datapack> empty;
      queue20 = empty;
      rule = false;

      wrenchBias = wrenchRaw;
      bias_pub.publish(wrenchBias);
      ROS_INFO("You called the bias.");

      vel_pub.publish(velFoward);
      usleep(300000);
      double a=0;
      double b=0;
      double c=0;
      int qSize = 0;

      while(!rule&&ros::ok())
      {
          //wrenchReal = wrenchSubstract(wrenchRaw, wrenchBias);
          nowWrench = wrenchNow;
          nowState = jointState;

          a = nowWrench.wrench.force.x;
          b = nowWrench.wrench.force.y;
          c = nowWrench.wrench.force.z;
          collisionForce = sqrt(a*a + b*b +c*c);
          rule = (collisionForce>collisonThreshold);
          tempPack = packAssign(nowState,nowWrench,collisionForce);
          qSize = queue20.size();

          if(qSize>250)// The force measure frequency is 250Hz, only reserves data in one second before reaching the collision threshold.
          {
              queue20.pop();
          }
          queue20.push(tempPack);
          usleep(3000);
      }
      vel_pub.publish(velBack);

      // 注意这个循环必须延时，不然采集不到力数据
      for(int n=0;n<50;n++)
      {
          nowWrench = wrenchNow;
          nowState = jointState;

          a = nowWrench.wrench.force.x;
          b = nowWrench.wrench.force.y;
          c = nowWrench.wrench.force.z;
          collisionForce = sqrt(a*a + b*b + c*c);
          tempPack = packAssign(nowState,nowWrench,collisionForce);
          queue20.push(tempPack);
          usleep(8000);
      }
      vel_pub.publish(velStop);
      usleep(100000);
      ROS_INFO("I got [%d] point.",explorePointNum);

      double movetime;
      recordPointInfo(queue20);
      movetime = calculateStep(queue20,stepCoefficient);

      nowDistance = nowDistance + movetime*moveSpeed;
      // 这里，加减速导致实际运动距离变小
      if((nowDistance)>exploreDistance)
          {
          break;
      }
      vel_pub.publish(velMove);
      usleep(movetime*1000000);
      vel_pub.publish(velStop);
      usleep(200000);
      ur_arm::PoseMatrix nowPose;
      nowPose = fKine(jointState.position);

      nowDistance = sqrt((nowPose.p[0] - startpose.p[0])*(nowPose.p[0] - startpose.p[0])\
              + (nowPose.p[1] - startpose.p[1])*(nowPose.p[1] - startpose.p[1])\
              + (nowPose.p[2] - startpose.p[2])*(nowPose.p[2] - startpose.p[2]));
  }
  vel_pub.publish(velMoveReverse);
  int backTime;
  backTime = fabs(exploreDistance/moveReverseSpeed);
  sleep(backTime);
  vel_pub.publish(velStop);
  usleep(200000);
  ROS_INFO("Exploration finished.");
  ROS_INFO("I have generated the jointsRecord.txt file.");
  ros::spin();
  return 0;
}

void exTorGet(geometry_msgs::WrenchStamped awrench)
{
    wrenchNow = awrench;
}

void biasGET(geometry_msgs::WrenchStamped awrench)
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
    vy = -exploreSpeed;
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
    vy = backSpeed;
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
    vx = moveSpeed;
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
    vx = -moveReverseSpeed;
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
    jointState = curState;
}

void recordPointInfo(queue<datapack> qq)
{
    queue<datapack> pp;
    pp = qq;
    int size;
    size = pp.size();
    for(int i=0;i<size;i++)
    {
        datapack dp;
        dp = pp.front();
        pp.pop();
        if(dp.forceAll>collisonThreshold)
        {
            fout1<<dp.robotState.header.stamp<<",";
            for(int j=0;j<6;j++)
            {
                fout1<<dp.robotState.position[j]<<",";
            }
            fout1<<dp.sensorData.wrench.force.x<<",";
            fout1<<dp.sensorData.wrench.force.y<<",";
            fout1<<dp.sensorData.wrench.force.z<<",";
            fout1<<dp.sensorData.wrench.torque.x<<",";
            fout1<<dp.sensorData.wrench.torque.y<<",";
            fout1<<dp.sensorData.wrench.torque.z<<",";
            fout1<<dp.forceAll<<endl;
            break;
        }
    }
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

datapack packAssign(sensor_msgs::JointState js, geometry_msgs::WrenchStamped ws, double db)
{
    datapack a;
    a.robotState = js;
    a.sensorData = ws;
    a.forceAll = db;
    return a;
}

double calculateStep(queue<datapack> qq, double cc)
{
    queue<datapack> pp;
    pp = qq;

    int size;
    double firstcontact;
    double firstcontactTime;
    double firstpass;
    double firstpassTime;

    ur_arm::PoseMatrix pose;

    size = pp.size();
    datapack ttime1;
    datapack ttime2;
    ttime1 = pp.front();
    ttime2 = pp.back();
    cout<<"Start time of the pack is: "<<ttime1.robotState.header.stamp<<endl;
    cout<<"End time of the pack is: "<<ttime2.robotState.header.stamp<<endl;

    for(int i=0;i<size;i++)
    {
        datapack dp1;
        dp1 = pp.front();
        pp.pop();
        if(dp1.forceAll>contactForce)
        {
            firstcontact = dp1.forceAll;
            firstcontactTime = dp1.robotState.header.stamp.toSec();
            break;
        }
    }

    for(int i=0;i<size;i++)
    {
        datapack dp2;
        dp2 = pp.front();
        pp.pop();
        if(dp2.forceAll>collisonThreshold)
        {
            firstpass = dp2.forceAll;
            firstpassTime = dp2.robotState.header.stamp.toSec();
            break;
        }
    }

    double movetime;
    double virtualK;

    ROS_INFO("First contact force is [%lf].",firstcontact);
    ROS_INFO("First pass force is [%lf].",firstpass);
    virtualK = fabs((firstpass-firstcontact)/((firstpassTime-firstcontactTime)*exploreSpeed));


    movetime = cc*(coef_a*virtualK + coef_b);
    if(movetime < minMoveTime)
        movetime = minMoveTime;
    if(movetime > maxMoveTime)
        movetime = maxMoveTime;

    //movetime = rigidMoveTime; // add this to let the movetime be a rigid value

    fout2<<virtualK<<endl;

    ROS_INFO("data size is [%d].",size);
    ROS_INFO("Virtual stiffness in this point is [%lf].",virtualK);
    ROS_INFO("Move time is [%lf] s.",movetime);
    return movetime;
}
