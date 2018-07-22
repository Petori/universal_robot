//　Created by Petori in 20180712
// My func definition.
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>
#include <math.h>
#include <cstdlib>
#include <Eigen/Dense>
#include "ur_arm/PoseMatrix.h"
#include "ur_arm/AllAng.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

double pi = 3.1415926;

double signsign(double x)
{
        if (x > 1e-10)
                x = 1;
        else if (x < -1e-10)
                x = -1;
        else
                x = 0;
        return x;
}

void hi()
{
    cout<< "Hello!" << endl;
}

double reZero(double x)
{
    // fabs(*)<1e-5 is the best , do not change it.
    if (fabs(x)<1e-3)
    {
        x = 0;
    }
    return x;
}

double reZeroForVel(double x)
{
    if (fabs(x)<0.02)
    {
        x = 0;
    }
    return x;
}

ur_arm::PoseMatrix fKine(std::vector<double> pos)
{
    // foward kinematics

    ur_arm::PoseMatrix pose;

    double d1=0.0892;
    double a2=-0.425;
    double a3=-0.39243;
    double d4=0.109;
    double d5=0.093;
    double d6=0.082;
    double theta1,theta2,theta3,theta4,theta5,theta6,c1,c2,c3,c4,c5,c6,s1,s2,s3,s4,s5,s6,c234,s234,c23,s23;

    theta1 = pos[0];
    theta2 = pos[1];
    theta3 = pos[2];
    theta4 = pos[3];
    theta5 = pos[4];
    theta6 = pos[5];

    c1=reZero(cos(theta1));
    s1=reZero(sin(theta1));
    c2=reZero(cos(theta2));
    s2=reZero(sin(theta2));
    c3=reZero(cos(theta3));
    s3=reZero(sin(theta3));
    c4=reZero(cos(theta4));
    s4=reZero(sin(theta4));
    c5=reZero(cos(theta5));
    s5=reZero(sin(theta5));
    c6=reZero(cos(theta6));
    s6=reZero(sin(theta6));

    c234=reZero(cos(theta2+theta3+theta4));
    s234=reZero(sin(theta2+theta3+theta4));

    c23=reZero(cos(theta2+theta3));
    s23=reZero(sin(theta2+theta3));

    pose.n[0] = c6*(c1*c5*c234+s1*s5)-c1*s234*s6;
    pose.o[0] = -s6*(c1*c5*c234+s1*s5)-c1*s234*c6;
    pose.a[0] = -c1*s5*c234+s1*c5;

    pose.n[1] = c6*(s1*c5*c234-c1*s5)-s1*s234*s6;
    pose.o[1] = -s6*(s1*c5*c234-c1*s5)-s1*s234*c6;
    pose.a[1] = -s1*s5*c234-c1*c5;

    pose.n[2] = s234*c5*c6+c234*s6;
    pose.o[2] = -s234*c5*s6+c234*c6;
    pose.a[2] = -s234*s5;

    double nnn,ooo,aaa;
    nnn = pose.n[0]*pose.n[0] + pose.n[1]*pose.n[1] + pose.n[2]*pose.n[2];
    ooo = pose.o[0]*pose.o[0] + pose.o[1]*pose.o[1] + pose.o[2]*pose.o[2];
    aaa = pose.a[0]*pose.a[0] + pose.a[1]*pose.a[1] + pose.a[2]*pose.a[2];
    for(int i=0;i<3;i++)
    {
        pose.n[i] = pose.n[i]/nnn;
        pose.o[i] = pose.o[i]/ooo;
        pose.a[i] = pose.a[i]/aaa;
    }
    pose.p[0] = d6*(-c1*s5*c234+s1*c5)+d5*c1*s234+d4*s1+a3*c1*c23+a2*c1*c2;
    pose.p[1] = d6*(-s1*s5*c234-c1*c5)+d5*s1*s234-d4*c1+a3*s1*c23+a2*s1*c2;
    pose.p[2] = -d6*s234*s5-d5*c234+a3*s23+a2*s2+d1;

    return pose;
}

double thetaMod(double x)
{
    if (x<-pi)
    {
        return (x+2*pi);
    }
    else if (x==-pi)
    {
        return pi;
    }
    else if (x>pi)
    {
        return (x-2*pi);
    }
    else
    {
        return x;
    }
}

vector<double> deltaVelCacByJacob(std::vector<double> pos, std::vector<double> cartesianDeltaVel)
{
    double c1,c2,c3,c4,c5,c6,s1,s2,s3,s4,s5,s6;
    double d1=0.0892,d4=0.109,d5=0.093,d6=0.082;
    double a2=-0.425,a3=-0.39243;
    Eigen::MatrixXf Jacob(6,6);
    Eigen::MatrixXf cdvel(6,1);
    Eigen::MatrixXf velResult(6,1);
    std::vector<double> deltaVel;

    for (int i=0;i<6;i++)
    {
        cdvel(i,0) = cartesianDeltaVel[i];
    }

    c1 = reZero(cos(pos[0]));
    c2 = reZero(cos(pos[1]));
    c3 = reZero(cos(pos[2]));
    c4 = reZero(cos(pos[3]));
    c5 = reZero(cos(pos[4]));
    c6 = reZero(cos(pos[5]));

    s1 = reZero(sin(pos[0]));
    s2 = reZero(sin(pos[1]));
    s3 = reZero(sin(pos[2]));
    s4 = reZero(sin(pos[3]));
    s5 = reZero(sin(pos[4]));
    s6 = reZero(sin(pos[5]));

    Jacob = Eigen::MatrixXf::Zero(6,6);

    Jacob(0,0) = c1*(d4 + c5*d6) - s1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(0,1) = -c1*(a2*s2 + s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(0,2) = -c1*(s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(0,3) = c1*(d5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + d6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)));
    Jacob(0,4) = - d6*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2))*(c1*c5 + s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2))) - d6*s5*(c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3))*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3));
    Jacob(0,5) = 0;

    Jacob(1,0) = s1*(d4 + c5*d6) + c1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(1,1) = -s1*(a2*s2 + s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(1,2) = -s1*(s2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + c2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(1,3) = s1*(d5*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2)) + d6*s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)));
    Jacob(1,4) = d6*s5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3)) - d6*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)))*(c4*(c2*c3 - s2*s3) - s4*(c2*s3 + c3*s2));
    Jacob(1,5) = 0;

    Jacob(2,0) = 0;
    Jacob(2,1) = c1*(s1*(d4 + c5*d6) + c1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)))) - s1*(c1*(d4 + c5*d6) - s1*(a2*c2 + c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5))));
    Jacob(2,2) = c1*(s1*(d4 + c5*d6) + c1*c2*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) - c1*s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5))) - s1*(c1*(d4 + c5*d6) - c2*s1*(s3*(c4*d5 + d6*s4*s5) + a3*c3 + c3*(d5*s4 - c4*d6*s5)) + s1*s2*(s3*(d5*s4 - c4*d6*s5) + a3*s3 - c3*(c4*d5 + d6*s4*s5)));
    Jacob(2,3) = c1*(d5*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3)) - d6*s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)) + c5*d6*s1) - s1*(c1*c5*d6 - d5*(c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3)) + d6*s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2)));
    Jacob(2,4) = - d6*(c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3))*(c1*c5 + s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2))) - d6*(c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3))*(c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2)));
    Jacob(2,5) = 0;

    Jacob(3,0) = 0;
    Jacob(3,1) = s1;
    Jacob(3,2) = s1;
    Jacob(3,3) = s1;
    Jacob(3,4) = c4*(c1*c2*s3 + c1*c3*s2) + s4*(c1*c2*c3 - c1*s2*s3);
    Jacob(3,5) = c5*s1 - s5*(c4*(c1*c2*c3 - c1*s2*s3) - s4*(c1*c2*s3 + c1*c3*s2));

    Jacob(4,0) = 0;
    Jacob(4,1) = -c1;
    Jacob(4,2) = -c1;
    Jacob(4,3) = -c1;
    Jacob(4,4) = c4*(c2*s1*s3 + c3*s1*s2) + s4*(c2*c3*s1 - s1*s2*s3);
    Jacob(4,5) = - c1*c5 - s5*(c4*(c2*c3*s1 - s1*s2*s3) - s4*(c2*s1*s3 + c3*s1*s2));

    Jacob(5,0) = 1;
    Jacob(5,1) = 0;
    Jacob(5,2) = 0;
    Jacob(5,3) = 0;
    Jacob(5,4) = s4*(c2*s3 + c3*s2) - c4*(c2*c3 - s2*s3);
    Jacob(5,5) = -s5*(c4*(c2*s3 + c3*s2) + s4*(c2*c3 - s2*s3));

    velResult = Jacob.inverse()*cdvel;

    for (int i=0;i<6;i++)
    {
        deltaVel[i] = velResult(i,0);
    }

    return deltaVel;
}

ur_arm::AllAng invKine(ur_arm::PoseMatrix Tq)
{
    //% 反解函数------------------------------------------------------------------
    //% 根据末端位姿矩阵求解关节角--UR5
    //% 注意---------------------------------------------------------------------
    //% 本反解程序基于以下条件编写
    //% 1.UR5结构符合pieper准则
    //% 2.将UR机器人的关节角值域定为(-pi,pi]
    //% 3.当关节角求解结果十分接近0时，对其进行归零修正
    //% 4.当关节角theta5为0度或180度时，UR5机器人姿态奇异，对关节角解组做无解处理
    //% 5.UR5机器人的3号关节运动范围不是360度，具体参数未知。
    //%   现暂定theta3=[-165度,165度]（示教得到的大致数据），弧度大致为2.88rad
    //% 6.在某些情况下，机器人末端可能会与机器人自身发生碰撞
    //%   对于这种情况，本程序暂未处理
    //% 7.所有角度单位为rad，长度单位为m
    //% 以上---------------------------------------------------------------------

//    // use case
//    ur_arm::AllAng allang;
//    double n[3]={1,0,0};
//    double o[3]={0,1,0};
//    double a[3]={0,0,1};
//    double p[3]={0.3,0.3,0.3};
//    Tq.n[0] = n[0];Tq.n[1] = n[1];Tq.n[2] = n[2];
//    Tq.o[0] = o[0];Tq.o[1] = o[1];Tq.o[2] = o[2];
//    Tq.a[0] = a[0];Tq.a[1] = a[1];Tq.a[2] = a[2];
//    Tq.p[0] = p[0];Tq.p[1] = p[1];Tq.p[2] = p[2];
//    allang = invKine(Tq);

ur_arm::AllAng allang;

double d1,a2,a3,d4,d5,d6;
double nx,ny,nz,ox,oy,oz,ax,ay,az,px,py,pz;
double beta1,beta2,beta3,beta4;
double c2_1,c2_2,c2_3,c2_4,c2_5,c2_6,c2_7,c2_8,c5_1,c5_3,c6_1,c6_2,c6_3,c6_4,cbeta1,cbeta2,cbeta3,cbeta4;
double s2_1,s2_2,s2_3,s2_4,s2_5,s2_6,s2_7,s2_8,s5_1,s5_3,s6_1,s6_2,s6_3,s6_4,sbeta1,sbeta2,sbeta3,sbeta4;
double imn,sqrt1,L1_1,L1_3,L1_5,L1_7,L2_1,L2_3,L2_5,L2_7;
double mark1,mark2,mark3,mark4,mark5,mark6,mark7,mark8;

double theta1_1,theta1_2,theta2_1,theta2_2,theta2_3,theta2_4,theta2_5,theta2_6,theta2_7,theta2_8;
double theta3_1,theta3_2,theta3_3,theta3_4,theta3_5,theta3_6,theta3_7,theta3_8;
double theta4_1,theta4_2,theta4_3,theta4_4,theta4_5,theta4_6,theta4_7,theta4_8;
double theta5_1,theta5_2,theta5_3,theta5_4,theta6_1,theta6_2,theta6_3,theta6_4;

double angle1[6],angle2[6],angle3[6],angle4[6],angle5[6],angle6[6],angle7[6],angle8[6];

//% UR机器人的DH参数-Serie1系列
d1=0.0892;
a2=-0.425;
a3=-0.39243;
d4=0.109;
d5=0.093;
d6=0.082;

nx=Tq.n[0];ny=Tq.n[1];nz=Tq.n[2];
ox=Tq.o[0];oy=Tq.o[1];oz=Tq.o[2];
ax=Tq.a[0];ay=Tq.a[1];az=Tq.a[2];
px=Tq.p[0];py=Tq.p[1];pz=Tq.p[2];

sqrt1=pow((d6*ay-py),2)+pow((px-ax*d6),2)-d4*d4;
sqrt1=reZero(sqrt1);

theta1_1=atan2((d6*ay-py),(-px+ax*d6))-atan2(d4,sqrt(sqrt1));
theta1_2=atan2((d6*ay-py),(-px+ax*d6))-atan2(d4,-sqrt(sqrt1));


theta1_1=reZero(theta1_1);
theta1_2=reZero(theta1_2);

theta1_1=thetaMod(theta1_1);
theta1_2=thetaMod(theta1_2);

c5_1=sin(theta1_1)*ax-cos(theta1_1)*ay;
s5_1=sqrt(1-pow((c5_1),2));
theta5_1=atan2(s5_1,c5_1);
theta5_2=atan2(-s5_1,c5_1);

theta5_1=reZero(theta5_1);
theta5_2=reZero(theta5_2);
theta5_1=thetaMod(theta5_1);
theta5_2=thetaMod(theta5_2);

c5_3=sin(theta1_2)*ax-cos(theta1_2)*ay;
s5_3=sqrt(1-pow((c5_3),2));
theta5_3=atan2(s5_3,c5_3);
theta5_4=atan2(-s5_3,c5_3);

theta5_3=reZero(theta5_3);
theta5_4=reZero(theta5_4);
theta5_3=thetaMod(theta5_3);
theta5_4=thetaMod(theta5_4);

c6_1=(sin(theta1_1)*nx-cos(theta1_1)*ny)/sin(theta5_1);
s6_1=(cos(theta1_1)*oy-sin(theta1_1)*ox)/sin(theta5_1);
theta6_1=atan2(reZero(s6_1),reZero(c6_1));

c6_2=(sin(theta1_1)*nx-cos(theta1_1)*ny)/sin(theta5_2);
s6_2=(cos(theta1_1)*oy-sin(theta1_1)*ox)/sin(theta5_2);
theta6_2=atan2(reZero(s6_2),reZero(c6_2));

c6_3=(sin(theta1_2)*nx-cos(theta1_2)*ny)/sin(theta5_3);
s6_3=(cos(theta1_2)*oy-sin(theta1_2)*ox)/sin(theta5_3);
theta6_3=atan2(reZero(s6_3),reZero(c6_3));

c6_4=(sin(theta1_2)*nx-cos(theta1_2)*ny)/sin(theta5_4);
s6_4=(cos(theta1_2)*oy-sin(theta1_2)*ox)/sin(theta5_4);
theta6_4=atan2(reZero(s6_4),reZero(c6_4));

theta6_1=thetaMod(theta6_1);
theta6_2=thetaMod(theta6_2);
theta6_3=thetaMod(theta6_3);
theta6_4=thetaMod(theta6_4);

cbeta1=sin(theta6_1)*nz+cos(theta6_1)*oz;
sbeta1=cos(theta5_1)*(cos(theta6_1)*nz-sin(theta6_1)*oz)-sin(theta5_1)*az;
beta1=atan2(reZero(sbeta1),reZero(cbeta1));

cbeta2=sin(theta6_2)*nz+cos(theta6_2)*oz;
sbeta2=cos(theta5_2)*(cos(theta6_2)*nz-sin(theta6_2)*oz)-sin(theta5_2)*az;
beta2=atan2(reZero(sbeta2),reZero(cbeta2));

cbeta3=sin(theta6_3)*nz+cos(theta6_3)*oz;
sbeta3=cos(theta5_3)*(cos(theta6_3)*nz-sin(theta6_3)*oz)-sin(theta5_3)*az;
beta3=atan2(reZero(sbeta3),reZero(cbeta3));

cbeta4=sin(theta6_4)*nz+cos(theta6_4)*oz;
sbeta4=cos(theta5_4)*(cos(theta6_4)*nz-sin(theta6_4)*oz)-sin(theta5_4)*az;
beta4=atan2(reZero(sbeta4),reZero(cbeta4));

beta1=reZero(beta1);
beta2=reZero(beta2);
beta3=reZero(beta3);
beta4=reZero(beta4);

L1_1=cos(theta1_1)*px+sin(theta1_1)*py+d6*cos(beta1)*sin(theta5_1)-d5*sin(beta1);
L2_1=pz-d1+d6*sin(beta1)*sin(theta5_1)+d5*cos(beta1);
imn=fabs((pow((L1_1),2)+pow((L2_1),2)-a3*a3-a2*a2)/2/a3/a2);

if (imn>1)
{
    mark1=0;
    mark2=0;

    angle1[0] = 0; angle1[1] = -pi/2; angle1[2] = 0; angle1[3] = pi/2; angle1[4] = 0; angle1[5] = 0;
    angle2[0] = 0; angle2[1] = -pi/2; angle2[2] = 0; angle2[3] = pi/2; angle2[4] = 0; angle2[5] = 0;
}
else
{
    mark1=1;
    mark2=1;

    theta3_1=acos((pow((L1_1),2)+L2_1*L2_1-a3*a3-a2*a2)/2/a3/a2);
    theta3_2=-acos((pow((L1_1),2)+L2_1*L2_1-a3*a3-a2*a2)/2/a3/a2);

    theta3_1=reZero(theta3_1);
    theta3_2=reZero(theta3_2);
    theta3_1=thetaMod(theta3_1);
    theta3_2=thetaMod(theta3_2);

    c2_1=(L1_1*(a3*cos(theta3_1)+a2)+L2_1*a3*sin(theta3_1))/((pow((a3*cos(theta3_1)+a2),2)+pow((a3*sin(theta3_1)),2)));
    s2_1=(L2_1*(a3*cos(theta3_1)+a2)-L1_1*a3*sin(theta3_1))/((pow((a3*cos(theta3_1)+a2),2)+pow((a3*sin(theta3_1)),2)));
    c2_2=(L1_1*(a3*cos(theta3_2)+a2)+L2_1*a3*sin(theta3_2))/((pow((a3*cos(theta3_2)+a2),2)+pow((a3*sin(theta3_2)),2)));
    s2_2=(L2_1*(a3*cos(theta3_2)+a2)-L1_1*a3*sin(theta3_2))/((pow((a3*cos(theta3_2)+a2),2)+pow((a3*sin(theta3_2)),2)));
    theta2_1=atan2(s2_1,c2_1);
    theta2_2=atan2(s2_2,c2_2);

    theta2_1=reZero(theta2_1);
    theta2_2=reZero(theta2_2);
    theta2_1=thetaMod(theta2_1);
    theta2_2=thetaMod(theta2_2);

    theta4_1=beta1-theta2_1-theta3_1;
    theta4_2=beta1-theta2_2-theta3_2;

    theta4_1=reZero(theta4_1);
    theta4_2=reZero(theta4_2);
    theta4_1=thetaMod(theta4_1);
    theta4_2=thetaMod(theta4_2);

    angle1[0] = theta1_1; angle1[1] = theta2_1; angle1[2] = theta3_1; angle1[3] = theta4_1; angle1[4] = theta5_1; angle1[5] = theta6_1;
    angle2[0] = theta1_1; angle2[1] = theta2_2; angle2[2] = theta3_2; angle2[3] = theta4_2; angle2[4] = theta5_1; angle2[5] = theta6_1;
}

L1_3=cos(theta1_1)*px+sin(theta1_1)*py+d6*cos(beta2)*sin(theta5_2)-d5*sin(beta2);
L2_3=pz-d1+d6*sin(beta2)*sin(theta5_2)+d5*cos(beta2);

imn=fabs((L1_3*L1_3+L2_3*L2_3-a3*a3-a2*a2)/2/a3/a2);
if (imn>1)
{
    mark3=0;
    mark4=0;
    angle3[0] = 0; angle3[1] = -pi/2; angle3[2] = 0; angle3[3] = pi/2; angle3[4] = 0; angle3[5] = 0;
    angle4[0] = 0; angle4[1] = -pi/2; angle4[2] = 0; angle4[3] = pi/2; angle4[4] = 0; angle4[5] = 0;
}
else
{
    mark3=1;
    mark4=1;

    theta3_3=acos((L1_3*L1_3+L2_3*L2_3-a3*a3-a2*a2)/2/a3/a2);
    theta3_4=-acos((L1_3*L1_3+L2_3*L2_3-a3*a3-a2*a2)/2/a3/a2);

    theta3_3=reZero(theta3_3);
    theta3_4=reZero(theta3_4);
    theta3_3=thetaMod(theta3_3);
    theta3_4=thetaMod(theta3_4);

    c2_3=(L1_3*(a3*cos(theta3_3)+a2)+L2_3*a3*sin(theta3_3))/((pow((a3*cos(theta3_3)+a2),2)+pow((a3*sin(theta3_3)),2)));
    s2_3=(L2_3*(a3*cos(theta3_3)+a2)-L1_3*a3*sin(theta3_3))/((pow((a3*cos(theta3_3)+a2),2)+pow((a3*sin(theta3_3)),2)));
    c2_4=(L1_3*(a3*cos(theta3_4)+a2)+L2_3*a3*sin(theta3_4))/((pow((a3*cos(theta3_4)+a2),2)+pow((a3*sin(theta3_4)),2)));
    s2_4=(L2_3*(a3*cos(theta3_4)+a2)-L1_3*a3*sin(theta3_4))/((pow((a3*cos(theta3_4)+a2),2)+pow((a3*sin(theta3_4)),2)));
    theta2_3=atan2(s2_3,c2_3);
    theta2_4=atan2(s2_4,c2_4);

    theta2_3=reZero(theta2_3);
    theta2_4=reZero(theta2_4);
    theta2_3=thetaMod(theta2_3);
    theta2_4=thetaMod(theta2_4);

    theta4_3=beta2-theta2_3-theta3_3;
    theta4_4=beta2-theta2_4-theta3_4;

    theta4_3=reZero(theta4_3);
    theta4_4=reZero(theta4_4);
    theta4_3=thetaMod(theta4_3);
    theta4_4=thetaMod(theta4_4);

    angle3[0] = theta1_1; angle3[1] = theta2_3; angle3[2] = theta3_3; angle3[3] = theta4_3; angle3[4] = theta5_2; angle3[5] = theta6_2;
    angle4[0] = theta1_1; angle4[1] = theta2_4; angle4[2] = theta3_4; angle4[3] = theta4_4; angle4[4] = theta5_2; angle4[5] = theta6_2;
}

L1_5=cos(theta1_2)*px+sin(theta1_2)*py+d6*cos(beta3)*sin(theta5_3)-d5*sin(beta3);
L2_5=pz-d1+d6*sin(beta3)*sin(theta5_3)+d5*cos(beta3);

imn=fabs((L1_5*L1_5+L2_5*L2_5-a3*a3-a2*a2)/2/a3/a2);
if (imn>1)
{
    mark5=0;
    mark6=0;
    angle5[0] = 0; angle5[1] = -pi/2; angle5[2] = 0; angle5[3] = pi/2; angle5[4] = 0; angle5[5] = 0;
    angle6[0] = 0; angle6[1] = -pi/2; angle6[2] = 0; angle6[3] = pi/2; angle6[4] = 0; angle6[5] = 0;
}
else
{
    mark5=1;
    mark6=1;

    theta3_5=acos((L1_5*L1_5+L2_5*L2_5-a3*a3-a2*a2)/2/a3/a2);
    theta3_6=-acos((L1_5*L1_5+L2_5*L2_5-a3*a3-a2*a2)/2/a3/a2);

    theta3_5=reZero(theta3_5);
    theta3_6=reZero(theta3_6);
    theta3_5=thetaMod(theta3_5);
    theta3_6=thetaMod(theta3_6);

    c2_5=(L1_5*(a3*cos(theta3_5)+a2)+L2_5*a3*sin(theta3_5))/((pow((a3*cos(theta3_5)+a2),2)+pow((a3*sin(theta3_5)),2)));
    s2_5=(L2_5*(a3*cos(theta3_5)+a2)-L1_5*a3*sin(theta3_5))/((pow((a3*cos(theta3_5)+a2),2)+pow((a3*sin(theta3_5)),2)));
    c2_6=(L1_5*(a3*cos(theta3_6)+a2)+L2_5*a3*sin(theta3_6))/((pow((a3*cos(theta3_6)+a2),2)+pow((a3*sin(theta3_6)),2)));
    s2_6=(L2_5*(a3*cos(theta3_6)+a2)-L1_5*a3*sin(theta3_6))/((pow((a3*cos(theta3_6)+a2),2)+pow((a3*sin(theta3_6)),2)));
    theta2_5=atan2(s2_5,c2_5);
    theta2_6=atan2(s2_6,c2_6);

    theta2_5=reZero(theta2_5);
    theta2_6=reZero(theta2_6);
    theta2_5=thetaMod(theta2_5);
    theta2_6=thetaMod(theta2_6);

    theta4_5=beta3-theta2_5-theta3_5;
    theta4_6=beta3-theta2_6-theta3_6;

    theta4_5=reZero(theta4_5);
    theta4_6=reZero(theta4_6);
    theta4_5=thetaMod(theta4_5);
    theta4_6=thetaMod(theta4_6);

    angle5[0] = theta1_2; angle5[1] = theta2_5; angle5[2] = theta3_5; angle5[3] = theta4_5; angle5[4] = theta5_3; angle5[5] = theta6_3;
    angle6[0] = theta1_2; angle6[1] = theta2_6; angle6[2] = theta3_6; angle6[3] = theta4_6; angle6[4] = theta5_3; angle6[5] = theta6_3;
}

L1_7=cos(theta1_2)*px+sin(theta1_2)*py+d6*cos(beta4)*sin(theta5_4)-d5*sin(beta4);
L2_7=pz-d1+d6*sin(beta4)*sin(theta5_4)+d5*cos(beta4);

imn=fabs((L1_7*L1_7+L2_7*L2_7-a3*a3-a2*a2)/2/a3/a2);
if (imn>1)
{
    mark7=0;
    mark8=0;
    angle7[0] = 0; angle7[1] = -pi/2; angle7[2] = 0; angle7[3] = pi/2; angle7[4] = 0; angle7[5] = 0;
    angle8[0] = 0; angle8[1] = -pi/2; angle8[2] = 0; angle8[3] = pi/2; angle8[4] = 0; angle8[5] = 0;
}
else
{
    mark7=1;
    mark8=1;

    theta3_7=acos((L1_7*L1_7+L2_7*L2_7-a3*a3-a2*a2)/2/a3/a2);
    theta3_8=-acos((L1_7*L1_7+L2_7*L2_7-a3*a3-a2*a2)/2/a3/a2);

    theta3_7=reZero(theta3_7);
    theta3_8=reZero(theta3_8);
    theta3_7=thetaMod(theta3_7);
    theta3_8=thetaMod(theta3_8);

    c2_7=(L1_7*(a3*cos(theta3_7)+a2)+L2_7*a3*sin(theta3_7))/((pow((a3*cos(theta3_7)+a2),2)+pow((a3*sin(theta3_7)),2)));
    s2_7=(L2_7*(a3*cos(theta3_7)+a2)-L1_7*a3*sin(theta3_7))/((pow((a3*cos(theta3_7)+a2),2)+pow((a3*sin(theta3_7)),2)));
    c2_8=(L1_7*(a3*cos(theta3_8)+a2)+L2_7*a3*sin(theta3_8))/((pow((a3*cos(theta3_8)+a2),2)+pow((a3*sin(theta3_8)),2)));
    s2_8=(L2_7*(a3*cos(theta3_8)+a2)-L1_7*a3*sin(theta3_8))/((pow((a3*cos(theta3_8)+a2),2)+pow((a3*sin(theta3_8)),2)));
    theta2_7=atan2(s2_7,c2_7);
    theta2_8=atan2(s2_8,c2_8);

    theta2_7=reZero(theta2_7);
    theta2_8=reZero(theta2_8);
    theta2_7=thetaMod(theta2_7);
    theta2_8=thetaMod(theta2_8);

    theta4_7=beta4-theta2_7-theta3_7;
    theta4_8=beta4-theta2_8-theta3_8;

    theta4_7=reZero(theta4_7);
    theta4_8=reZero(theta4_8);
    theta4_7=thetaMod(theta4_7);
    theta4_8=thetaMod(theta4_8);

    angle7[0] = theta1_2; angle7[1] = theta2_7; angle7[2] = theta3_7; angle7[3] = theta4_7; angle7[4] = theta5_4; angle7[5] = theta6_4;
    angle8[0] = theta1_2; angle8[1] = theta2_8; angle8[2] = theta3_8; angle8[3] = theta4_8; angle8[4] = theta5_4; angle8[5] = theta6_4;
}

if (fabs(angle1[2])>2.88)
{mark1=0;}

if (fabs(angle2[2])>2.88)
{mark2=0;}

if (fabs(angle3[2])>2.88)
{mark3=0;}

if (fabs(angle4[2])>2.88)
{mark4=0;}

if (fabs(angle7[2])>2.88)
{mark5=0;}

if (fabs(angle6[2])>2.88)
{mark6=0;}

if (fabs(angle7[2])>2.88)
{mark7=0;}

if (fabs(angle8[2])>2.88)
{mark8=0;}

for(int i=0;i<6;i++)
{
    allang.ang1[i] = angle1[i];
    allang.ang2[i] = angle2[i];
    allang.ang3[i] = angle3[i];
    allang.ang4[i] = angle4[i];
    allang.ang5[i] = angle5[i];
    allang.ang6[i] = angle6[i];
    allang.ang7[i] = angle7[i];
    allang.ang8[i] = angle8[i];
}
allang.ang1[6] = mark1;
allang.ang2[6] = mark2;
allang.ang3[6] = mark3;
allang.ang4[6] = mark4;
allang.ang5[6] = mark5;
allang.ang6[6] = mark6;
allang.ang7[6] = mark7;
allang.ang8[6] = mark8;

return allang;
}

void checkMyCode()
{

//    startPoint.push_back(1);
//    startPoint.push_back(1);
//    startPoint.push_back(1);
//    startPoint.push_back(1);
//    startPoint.push_back(1);
//    startPoint.push_back(1);


//    ur_arm::PoseMatrix startPose;
//    ur_arm::AllAng allangForJudge;

//    startPose = fKine(startPoint);
//    allangForJudge = invKine(startPose);

//    for(int i=0;i<3;i++)  cout<<startPose.n[i]<<" ";cout<<endl;
//    for(int i=0;i<3;i++)  cout<<startPose.o[i]<<" ";cout<<endl;
//    for(int i=0;i<3;i++)  cout<<startPose.a[i]<<" ";cout<<endl;
//    for(int i=0;i<3;i++)  cout<<startPose.p[i]<<" ";cout<<endl;

//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang1[i]<<" ";cout<<endl;
//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang2[i]<<" ";cout<<endl;
//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang3[i]<<" ";cout<<endl;
//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang4[i]<<" ";cout<<endl;
//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang5[i]<<" ";cout<<endl;
//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang6[i]<<" ";cout<<endl;
//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang7[i]<<" ";cout<<endl;
//    for(int i=0;i<7;i++)  cout<<allangForJudge.ang8[i]<<" ";cout<<endl;
}

void showPoseMatrix(ur_arm::PoseMatrix matrix)
{
    cout<<"The pose matrix is:"<<endl;
    cout<<" ["<<matrix.n[0]<<", "<<matrix.o[0]<<", "<<matrix.a[0]<<", "<<matrix.p[0]<<endl;
    cout<<"  "<<matrix.n[1]<<", "<<matrix.o[1]<<", "<<matrix.a[1]<<", "<<matrix.p[1]<<endl;
    cout<<"  "<<matrix.n[2]<<", "<<matrix.o[2]<<", "<<matrix.a[2]<<", "<<matrix.p[2]<<endl;
    cout<<"  "<<0<<", "<<0<<", "<<0<<", "<<1<<"]"<<endl;
}
