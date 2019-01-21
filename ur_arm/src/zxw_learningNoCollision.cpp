// 2018/10/26---
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"

#define random(x) (rand()%x)

// global variables
std::vector<double> curPos;
std::vector<double> curVel;
std::vector<double> curEff;
// fuction definition
void getJointState(sensor_msgs::JointState curState);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vel_control");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher chatter_pub = n.advertise<ur_arm::Joints>("/ur_arm/cmd_joint_vel", 1);
  // Subscribing the joint_states and record them.
   ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, getJointState);
  sleep(1.0);
  ur_arm::Joints vel_move;
  ur_arm::Joints vel_static;

  double desired_pos;
  double desired_vol;
  // double k_p= -0.5;
  // double k_v= -0.5;
  double u_control;
  double  u_control_M1;
  double H_22;

  int Num;
  Eigen::MatrixXf K_control(1,4);
  Eigen::MatrixXf P_i(15,15);
  Eigen::MatrixXf Theta(15,1);
  Eigen::MatrixXf X_state(4,1);
  Eigen::MatrixXf X_state_M1(4,1);
  Eigen::MatrixXf H_xx_M1(5,1);
  Eigen::MatrixXf H_xx(5,1);
  Eigen::MatrixXf Q_1(4,4);
  Eigen::MatrixXf Q_k_M1(15,1);
  Eigen::MatrixXf Q_k(15,1);
  Eigen::MatrixXf phi_e(15,1);
  Eigen::MatrixXf Trans_phi_e(1,15);
  Eigen::MatrixXf H_21(1,4);
  Eigen::MatrixXf temp(1,1);
  Eigen::MatrixXf c_t(1,1);
  Eigen::MatrixXf eye(1,1);
    eye(0,0) = 1;

  K_control(0,0)=0;K_control(0,1)=0;K_control(0,2)=0;K_control(0,3)=0;
  for(int i =0;i<15;i++) {
          for(int j=0;j<15;j++) {
            P_i(i,j)=0;
            if(i==j) {P_i(i,j)=100;}
          }
          }
for(int i =0;i<15;i++)
      {
      Theta(i,0)=0;
      }
for(int i =0;i<4;i++) {
        for(int j=0;j<4;j++) {
          Q_1(i,j)=0;
          if(i==j) {Q_1(i,j)=3;}
        }
        }
Q_1(0,2)=-3;
Q_1(1,3)=-3;
Q_1(2,0)=-3;
Q_1(3,1)=-3;

  desired_pos=2;
  desired_vol=0;
  X_state(0,0)=0;
  X_state(1,0)=0;
  X_state(2,0)=0;
  X_state(3,0)=0;

  u_control=0;
  K_control(0,0)=-0.1;
  K_control(0,1)=-0.1;
  K_control(0,2)=0.1;
  K_control(0,3)=0.1;


for(int iii=0;iii<6;iii++){
      if(K_control(0,0)>0.1) {K_control(0,0)=-0.1;}
      int times = 0;
      while(times<1250&&ros::ok())
      {
          usleep(8000);
          times++;


          X_state_M1(0,0)= X_state(0,0);      X_state_M1(1,0)= X_state(1,0);
          X_state_M1(2,0)= X_state(2,0);      X_state_M1(3,0)= X_state(3,0);
          u_control_M1= u_control;
          X_state(0,0)=curPos[0];  X_state(1,0)=curVel[0];
          X_state(2,0)=desired_pos;      X_state(3,0)=desired_vol;
          temp =  K_control*X_state;

          u_control=temp(0,0)+ (double(random(100))-50)/100;
          vel_move.base = u_control;

          H_xx_M1(0,0)=X_state_M1(0,0);    H_xx_M1(1,0)=X_state_M1(1,0);
          H_xx_M1(2,0)=X_state_M1(2,0);    H_xx_M1(3,0)=X_state_M1(3,0);
          H_xx_M1(4,0)=u_control_M1;

          H_xx(0,0)=X_state(0,0);    H_xx(1,0)=X_state(1,0);
          H_xx(2,0)=X_state(2,0);    H_xx(3,0)=X_state(3,0);
	  temp=K_control*X_state_M1;
          H_xx(4,0)=temp(0,0);

          temp(0,0) = u_control_M1*u_control_M1;
          c_t=  X_state_M1.transpose()*Q_1*X_state_M1+ temp;
          Num=0;
          for(int i =0;i<5;i++) {
                  for(int j=0;j<5;j++) {
                    Q_k_M1(Num,0)=0;
                    Q_k(Num,0)=0;
                    if(i>=j) {
                        Q_k_M1(Num,0)=H_xx_M1(i,0)*H_xx_M1(j,0);
                        Q_k(Num,0)=H_xx(i,0)*H_xx(j,0);
                        Num=Num+1;
                                }
                  }
                  }
          phi_e=Q_k_M1-Q_k;
          Trans_phi_e=phi_e.transpose();

          temp = (eye+Trans_phi_e*P_i*phi_e);
          Theta=Theta+P_i*phi_e*(c_t-Trans_phi_e*Theta)/temp(0,0);
          P_i=P_i-P_i*phi_e*Trans_phi_e*P_i/temp(0,0);

          vel_move.shoulder =0.;
          vel_move.elbow = 0.0;
          vel_move.wrist1 = 0.0;
          vel_move.wrist2 = 0.0;
          vel_move.wrist3 = 0.0;
          chatter_pub.publish(vel_move);
      }
      H_21(0,0)=Theta(4,0)/2;
      H_21(0,1)=Theta(8,0)/2;
      H_21(0,2)=Theta(11,0)/2;
      H_21(0,3)=Theta(13,0)/2;

      H_22=Theta(14,0);
      K_control=-H_21/H_22;
      ROS_INFO("K_control");
      std::cout<<K_control(0,0)<<"   "<<K_control(0,1)<<"   "<<K_control(0,2)<<"   "<<K_control(0,3)<<"   "<<std::endl;


    //  ROS_INFO("Joint 1 .");
    //  std::cout<<curPos[0]<<std::endl;
      chatter_pub.publish(vel_static);
      usleep(5000);

      int timesback = 0;
      while(timesback<1250&&ros::ok())
      {
          usleep(8000);
          timesback++;
        u_control=-1*(curPos[0]-0)-1*(curVel[0]-0);
        vel_move.base = u_control;
        vel_move.shoulder =0.;
        vel_move.elbow = 0.;
        vel_move.wrist1 = 0.0;
        vel_move.wrist2 = 0.0;
        vel_move.wrist3 = 0.0;
        chatter_pub.publish(vel_move);
        }
      chatter_pub.publish(vel_static);
      usleep(5000);
}
  ROS_INFO("Stopped.");

  return 0;
}

void getJointState(sensor_msgs::JointState curState)
{
    curPos = curState.position;
    curVel = curState.velocity;
//    std::cout<<curPos[0]<<std::endl;
//    std::cout<<curVel[0]<<std::endl;
}
