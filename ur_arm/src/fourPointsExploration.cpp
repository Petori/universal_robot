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
std::ofstream fout1("data/fourPoints.txt");
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
double collisonThreshold =3;

double stayTime = 1;
double moveSpeed_a = 0.03;// x direction
double moveTime_a = 2;
double moveSpeed_b = -0.03;// z direction
double moveTime_b = 1;
double exploreSpeed = -0.02;// y direction
double backSpeed = 0.05;// y direction
double backTime = 1;


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
  ros::init(argc, argv, "Four_points_exploration");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/ur_arm/cmd_tool_vel", 1);
  ros::Publisher bias_pub = n.advertise<geometry_msgs::WrenchStamped>("/bias_dealt_data",1);
  ros::Subscriber collisionCheck = n.subscribe<geometry_msgs::WrenchStamped>("/dealt_data", 1, exTorGet);
  ros::Subscriber sub1 = n.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, biasGET);
  ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, jointStateGet);
  usleep(500000);//Leave 0.5s for building the subscribers and publishers

  // Velocities initation
  geometry_msgs::Twist vel1;
  geometry_msgs::Twist vel2;
  geometry_msgs::Twist vel3;
  geometry_msgs::Twist vel4;
  geometry_msgs::Twist velStop;
  geometry_msgs::Twist velExplore;
  geometry_msgs::Twist velBack;

  vel1.linear.x = moveSpeed_a;
  vel2.linear.z = moveSpeed_b;
  vel3.linear.x = -moveSpeed_a;
  vel4.linear.z = -moveSpeed_b;
  velExplore.linear.y = exploreSpeed;
  velBack.linear.y = backSpeed;

  // for servcie call bias ---- bias the topic /transformed_word
  netft_utils::SetBias bias;
  bias.request.toBias = true;
  bias.request.forceMax = 50;
  bias.request.torqueMax = 10;

  // process data

  queue<datapack> empty;
  queue<datapack> queue20;
  datapack tempPack;
  sensor_msgs::JointState nowState;
  geometry_msgs::WrenchStamped nowWrench;

  bool rule;

  double a=0;
  double b=0;
  double c=0;

  queue20 = empty;

//  double distanceInterval = 0.04; So the move time is 2s.
  signal(SIGINT, Stop);// deal with the "ctrl + C"



  /*************************Exploration starts**********************/
  ROS_INFO("Exploration starts.");
  sleep(1);

  /*************Point 1*************/

  rule = false;

  wrenchBias = wrenchRaw;
  bias_pub.publish(wrenchBias);
  ROS_INFO("You called the bias.");

  vel_pub.publish(velExplore);
  sleep(1);

  while(!rule&&ros::ok())
  {
      nowWrench = wrenchNow;
      nowState = jointState;

      a = nowWrench.wrench.force.x;
      b = nowWrench.wrench.force.y;
      c = nowWrench.wrench.force.z;

      collisionForce = sqrt(a*a + b*b +c*c);

      rule = (collisionForce>collisonThreshold);

      tempPack = packAssign(nowState,nowWrench,collisionForce);

      if(queue20.size()>10)
      {
          queue20.pop();
      }
      queue20.push(tempPack);
  }
  vel_pub.publish(velBack);

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
  recordPointInfo(queue20);
  sleep(backTime);

  vel_pub.publish(velStop);
  ROS_INFO("I got point 1.");
  sleep(stayTime);


  /*************Point 2*************/

  vel_pub.publish(vel1);
  sleep(moveTime_a);
  vel_pub.publish(velStop);
  sleep(stayTime);

  rule = false;

  wrenchBias = wrenchRaw;
  bias_pub.publish(wrenchBias);
  ROS_INFO("You called the bias.");

  vel_pub.publish(velExplore);
  sleep(1);

  while(!rule&&ros::ok())
  {
      nowWrench = wrenchNow;
      nowState = jointState;

      a = nowWrench.wrench.force.x;
      b = nowWrench.wrench.force.y;
      c = nowWrench.wrench.force.z;

      collisionForce = sqrt(a*a + b*b +c*c);

      rule = (collisionForce>collisonThreshold);

      tempPack = packAssign(nowState,nowWrench,collisionForce);

      if(queue20.size()>10)
      {
          queue20.pop();
      }
      queue20.push(tempPack);
  }
  vel_pub.publish(velBack);

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
  recordPointInfo(queue20);
  sleep(backTime);

  vel_pub.publish(velStop);
  ROS_INFO("I got point 2.");
  sleep(stayTime);

  /*************Point 3 *************/

  vel_pub.publish(vel2);
  sleep(moveTime_b);
  vel_pub.publish(velStop);
  sleep(stayTime);

  rule = false;

  wrenchBias = wrenchRaw;
  bias_pub.publish(wrenchBias);
  ROS_INFO("You called the bias.");

  vel_pub.publish(velExplore);
  sleep(1);

  while(!rule&&ros::ok())
  {
      nowWrench = wrenchNow;
      nowState = jointState;

      a = nowWrench.wrench.force.x;
      b = nowWrench.wrench.force.y;
      c = nowWrench.wrench.force.z;

      collisionForce = sqrt(a*a + b*b +c*c);

      rule = (collisionForce>collisonThreshold);

      tempPack = packAssign(nowState,nowWrench,collisionForce);

      if(queue20.size()>10)
      {
          queue20.pop();
      }
      queue20.push(tempPack);
  }
  vel_pub.publish(velBack);

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
  recordPointInfo(queue20);
  sleep(backTime);

  vel_pub.publish(velStop);
  ROS_INFO("I got point 3.");
  sleep(stayTime);

  /*************Point 4 *************/

  vel_pub.publish(vel3);
  sleep(moveTime_a);
  vel_pub.publish(velStop);
  sleep(stayTime);

  rule = false;

  wrenchBias = wrenchRaw;
  bias_pub.publish(wrenchBias);
  ROS_INFO("You called the bias.");

  vel_pub.publish(velExplore);
  sleep(1);

  while(!rule&&ros::ok())
  {
      nowWrench = wrenchNow;
      nowState = jointState;

      a = nowWrench.wrench.force.x;
      b = nowWrench.wrench.force.y;
      c = nowWrench.wrench.force.z;

      collisionForce = sqrt(a*a + b*b +c*c);

      rule = (collisionForce>collisonThreshold);

      tempPack = packAssign(nowState,nowWrench,collisionForce);

      if(queue20.size()>10)
      {
          queue20.pop();
      }
      queue20.push(tempPack);
  }
  vel_pub.publish(velBack);

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
  recordPointInfo(queue20);
  sleep(backTime);

  vel_pub.publish(velStop);
  ROS_INFO("I got point 4.");
  sleep(stayTime);

  /*********** Move to the initial position *************/

  vel_pub.publish(vel4);
  sleep(moveTime_b);
  vel_pub.publish(velStop);
  sleep(stayTime);

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
