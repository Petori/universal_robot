// 2018/12/17---
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int8.h>
#include "ur_arm/Joints.h"
#include "ur_arm/my_func.h"

#define random(x) (rand()%x)

// global variables
std::vector<double> curPos;
std::vector<double> curVel;
std::vector<double> curEff;
std::ofstream fout1("/home/e305/Project/Robot_uu_V1/data/saveData_move_1.txt");
std::ofstream fout2("/home/e305/Project/Robot_uu_V1/data/saveData_move_2.txt");
std::ofstream fout3("/home/e305/Project/Robot_uu_V1/data/saveData_move_3.txt");
std::ofstream fout4("/home/e305/Project/Robot_uu_V1/data/saveData_Torch.txt");
std::ofstream fout5("/home/e305/Project/Robot_uu_V1/data/saveData_move_0.txt");
Eigen::MatrixXd Torch(6,1);

// fuction definition
void getJointState(sensor_msgs::JointState curState);
void recordExternalTorqueToTxt(ur_arm::Joints joints);
void CalculateTorch(geometry_msgs::WrenchStamped awrench);

int main(int argc, char **argv)
{
        ros::init(argc, argv, "vel_control");
        ros::NodeHandle n;
        ros::AsyncSpinner spinner(1);
        spinner.start();

        ros::Publisher chatter_pub = n.advertise<ur_arm::Joints>("/ur_arm/cmd_joint_vel", 1);
        // Subscribing the joint_states and record them.
        ros::Subscriber recorder1 = n.subscribe<sensor_msgs::JointState>("/joint_states", 1, getJointState);
        ros::Subscriber recorder2 = n.subscribe<geometry_msgs::WrenchStamped>("/transformed_world", 1, CalculateTorch);
        sleep(1.0);
        ur_arm::Joints vel_move;
        ur_arm::Joints vel_static;


        double height;
        double dt;
        double R = 1;
        int Safe_open = 1;
        int Num;
        int Eps_max = 20;
        double K_max = 3;
        double u_control;
        double u_control_a;
        double H_22;
        double u_control_b;
        double u_control_c;
        double x_;
        double y_;
        double l_bar;
        double d_ang;
        double alpha;
        double beta;
        int size_track = 255;

        double f_x_a;
        double f_x_0;
        double k_joint;
        double k_joint_a;
        double f_x_b;
        double f_x_b_0;
        double f_x_c;
        double k_joint_b;
        double k_joint_c;

        Eigen::MatrixXf J(6, 6);
        Eigen::MatrixXd Torch_joint(6, 1);

        Eigen::MatrixXd x_track(size_track, 1);
        Eigen::MatrixXd d_xtrack(size_track, 1);
        Eigen::MatrixXd K_control(1, 4);
        Eigen::MatrixXd X_state(4, 1);
        Eigen::MatrixXd temp(1, 1);
        Eigen::MatrixXd error_sum(1, 1);

        Eigen::MatrixXd x_track_a(size_track, 1);
        Eigen::MatrixXd d_xtrack_a(size_track, 1);
        Eigen::MatrixXd K_control_a(1, 4);
        Eigen::MatrixXd X_state_a(4, 1);
        Eigen::MatrixXd temp_a(1, 1);
        Eigen::MatrixXd eye_a(1, 1);
        Eigen::MatrixXd error_sum_a(1, 1);
        eye_a(0, 0) = 1;

        Eigen::MatrixXd x_track_b(size_track, 1);
        Eigen::MatrixXd d_xtrack_b(size_track, 1);
        Eigen::MatrixXd K_control_b(1, 4);
        Eigen::MatrixXd X_state_b(4, 1);
        Eigen::MatrixXd temp_b(1, 1);
        Eigen::MatrixXd eye_b(1, 1);
        Eigen::MatrixXd error_sum_b(1, 1);
        eye_b(0, 0) = 1;

        Eigen::MatrixXd x_track_c(size_track, 1);
        Eigen::MatrixXd d_xtrack_c(size_track, 1);
        Eigen::MatrixXd K_control_c(1, 4);
        Eigen::MatrixXd X_state_c(4, 1);
        Eigen::MatrixXd temp_c(1, 1);
        Eigen::MatrixXd error_sum_c(1, 1);


        X_state << 0, 0, 0, 0;
        error_sum << 0;
        u_control = 0;
        K_control <<-0.315634, -0.185924, 0.309695, 0.792303;

        X_state_a << 0, 0, 0, 0;
        error_sum_a << 0;
        u_control_a = 0;
        //-0.118303 0.0633693 0.0772858   0.91162
        K_control_a <<-0.508285, -0.103863, 0.489004, 0.778006;
        //K_control <<-0.5, -.5, 0.5, 0.5;

        X_state_b << 0, 0, 0, 0;
        error_sum_b << 0;
        u_control_b = 0;

        K_control_b <<-0.3, -0.036452, 0.3, 0.680786;
        //K_control_b <<-0.5, -.5, 0.5, 0.5;

        X_state_c << 0, 0, 0, 0;
        error_sum_c << 0;
        u_control_c = 0;

        K_control_c <<-0.3, -0.036452, 0.3, 0.680786;

        k_joint_a=-0.08;
        k_joint_b=-0.30;
        k_joint_c=-0.50;
        ////////////////////////// designed trajectory
        l_bar = 0.4;
        dt = 0.08;
        d_xtrack(0, 0) = 0;
        d_xtrack_a(0, 0) = 0;
        d_xtrack_b(0, 0) = 0;
        d_xtrack_c(0, 0) = 0;
        for (int i = 0; i < (size_track-5); i++)
        {
            //x_ = 0.1 + 0.3*(i * dt / 10);
            //if (i>125){x_ = 0.4 - 0.3*(i * dt / 10-1);}
            x_ = 0.1*(1 + i * dt / 5);
            y_ = 0.3;
            d_ang = sqrt(x_*x_ + y_ * y_);
            alpha = atan(y_ / x_);
            beta = acos(d_ang / (2 * l_bar));

            x_track(i, 0) = -0.5 * sin(i*dt)-3.14/2;
            if (i > 0) { d_xtrack(i, 0) = (x_track(i, 0) - x_track(i - 1, 0)) / dt;}

            x_track_a(i, 0) =-3.14/2-0.5 * sin(i*dt);//-(alpha + beta);//0.5 * sin(i*dt )-3.14/2;//
            if (i > 0) { d_xtrack_a(i, 0) = (x_track_a(i, 0) - x_track_a(i - 1, 0)) / dt;}// (x_track(i, 0) - x_track(i - 1, 0)) / dt;
            x_track_b(i, 0) =-3.14/2; //2 * beta;//3.14/2-0.5 * sin(i*dt );//2 * sin(i*dt * 2) - 3.14 / 2;//
            if (i > 0) { d_xtrack_b(i, 0) = 0;}//(x_track_b(i, 0) - x_track_b(i - 1, 0)) / dt;

            x_track_c(i, 0) =0.0;
            if (i > 0) { d_xtrack_c(i, 0) = 0;}
        }

        Torch_joint = J.cast<double>().transpose() * Torch;
        f_x_0=Torch_joint(2,0);
        f_x_b_0=Torch_joint(3,0);
        ///////////////////////// end
                Safe_open = 1;
                int times = 0;
                while (times < 2500/10 && ros::ok() && Safe_open == 1)
                {
                        usleep(8000*10);
                        times++;

                        J = cacJacob(curPos);

                        Torch_joint = J.cast<double>().transpose() * Torch;

                        X_state(0, 0) = curPos[1];  X_state(1, 0) = curVel[1];
                        X_state(2, 0) = x_track(times, 0);      X_state(3, 0) = d_xtrack(times, 0);

                        temp = K_control * X_state;
                        u_control = temp(0, 0);
                        if (u_control > 1) { u_control = 1; }
                        if (u_control < -1) { u_control = -1; }
                        if (X_state(0, 0) > -0.5) { Safe_open = 0; }//-45/180*3.14
                        if (X_state(0, 0) < -2.6) { Safe_open = 0; } //-150/180*3.14 

                        //u_control=-1*(curPos[0]-0)-1*(curVel[0]-0);
                        X_state_a(0, 0) = curPos[2];  X_state_a(1, 0) = curVel[2];
                        X_state_a(2, 0) = x_track_a(times, 0);      X_state_a(3, 0) = d_xtrack_a(times, 0);
                        f_x_a=(Torch_joint(2,0));

                        temp_a = K_control_a * X_state_a;
                        u_control_a = temp_a(0, 0)+k_joint_a*f_x_a;
                        if (u_control_a > 1) { u_control_a = 1; }
                        if (u_control_a < -1) { u_control_a = -1; }
                        if (X_state_a(0, 0) > -0.5) { Safe_open = 0; }//-45/180*3.14
                        if (X_state_a(0, 0) < -2.6) { Safe_open = 0; } //-150/180*3.14

                        X_state_b(0, 0) = curPos[3];  X_state_b(1, 0) = curVel[3];
                        X_state_b(2, 0) = x_track_b(times, 0);      X_state_b(3, 0) = d_xtrack_b(times, 0);
                        f_x_b=(Torch_joint(3,0));

                        temp_b = K_control_b * X_state_b;
                        u_control_b = temp_b(0, 0)+k_joint_b*f_x_b;
                        if (u_control_b > 1) { u_control_b = 1; }
                        if (u_control_b < -1) { u_control_b = -1; }
                        if (X_state_b(0, 0) > 2.2) { Safe_open = 0; } //150/180*3.14
                        if (X_state_b(0, 0) < -2.2) { Safe_open = 0; } //-10/180*3.14

                        X_state_c(0, 0) = curPos[4];  X_state_c(1, 0) = curVel[4];
                        X_state_c(2, 0) = x_track_c(times, 0);      X_state_c(3, 0) = d_xtrack_c(times, 0);
                        f_x_c=(Torch_joint(4,0));

                        temp_c = K_control_c * X_state_c;
                        u_control_c = temp_c(0, 0)+k_joint_c*f_x_c;
                        if (u_control_c > 1) { u_control_c = 1; }
                        if (u_control_c < -1) { u_control_c = -1; }
                        if (X_state_c(0, 0) > 2.2) { Safe_open = 0; } //150/180*3.14
                        if (X_state_c(0, 0) < -2.2) { Safe_open = 0; } //-10/180*3.14


                        vel_move.base = 0.0;
                        vel_move.shoulder = u_control;
                        vel_move.elbow = u_control_a;
                        vel_move.wrist1 = u_control_b;
                        vel_move.wrist2 = u_control_c;
                        vel_move.wrist3 = 0.0;
                        //vel_move.wrist2 = 0;
                        chatter_pub.publish(vel_move);

                        ROS_INFO("State11");
                        //std::cout << X_state.transpose() << "   " << u_control << " " << times << " " << Torch(1,0) << " " << std::endl;
                        std::cout <<  Torch_joint(2,0)  << " " << Torch_joint(3,0)  << " " << Torch_joint(4,0) << " " << std::endl;


                        // save data state
                        fout1 << X_state_a.transpose() << "   " << u_control_a <<  " " << std::endl;

                        //ROS_INFO("State22");
                        //std::cout << X_state_b.transpose() << "   " << u_control_b << " " << times << " " << Safe_open << " " << std::endl;

                        // save data state
                        fout2 << X_state_b.transpose() << "   " << u_control_b <<  " " << std::endl;

                        fout3 << X_state_c.transpose() << "   " << u_control_c <<  " " << std::endl;
                        
                        fout4 << Torch_joint.transpose() << "   " << std::endl; 

                        fout5 << X_state.transpose() << "   " << u_control << " " << std::endl;

                }
                chatter_pub.publish(vel_static);
                usleep(5000);

        ROS_INFO("Stopped.");
        fout1.close();
        fout2.close();
        fout3.close();
        fout4.close();
        fout5.close();
        return 0;
}

void getJointState(sensor_msgs::JointState curState)
{
        curPos = curState.position;
        curVel = curState.velocity;
        //    std::cout<<curPos[0]<<std::endl;
        //    std::cout<<curVel[0]<<std::endl;
}

void CalculateTorch(geometry_msgs::WrenchStamped awrench)
{
        geometry_msgs::Vector3 f;
        geometry_msgs::Vector3 t;
        Eigen::MatrixXd J(6,6);
        Eigen::MatrixXd W(6,1);

        f = awrench.wrench.force;
        t = awrench.wrench.torque;
        //J = cacJacob(curPos);
        W(0, 0) = f.x;
        W(1, 0) = f.y;
        W(2, 0) = f.z;
        W(3, 0) = t.x;
        W(4, 0) = t.y;
        W(5, 0) = t.z;
        Torch=W;
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
        fout1 << "header:" << std::endl;
        fout1 << "  seq: " << "000000" << std::endl;
        fout1 << "  stamp:" << std::endl;
        fout1 << "    secs: " << "000000" << std::endl;
        fout1 << "    nsecs: " << "000000" << std::endl;
        fout1 << "  frame_id: \'\'" << std::endl;

        // write the name;
        fout1 << "name: [";
        for (int i = 0; i < (curName.size() - 1); ++i)
        {
                fout1 << '\'';
                fout1 << curName[i] << "\', ";
        }
        fout1 << '\'' << curName[curName.size() - 1] << '\'' << ']' << std::endl;

        // write the position;
        fout1 << "position: [";
        for (int i = 0; i < (curPos.size() - 1); ++i)
        {
                fout1 << curPos[i] << ", ";
        }
        fout1 << curPos[curPos.size() - 1] << ']' << std::endl;

        // write the velocity
        fout1 << "velocity: [";
        for (int i = 0; i < (curVel.size() - 1); ++i)
        {
                fout1 << curVel[i] << ", ";
        }
        fout1 << curVel[curVel.size() - 1] << ']' << std::endl;

        // write the effort
        fout1 << "effort: [";
        for (int i = 0; i < (curEff.size() - 1); ++i)
        {
                fout1 << curEff[i] << ", ";
        }
        fout1 << curEff[curEff.size() - 1] << ']' << std::endl;
        fout1 << "---" << std::endl;
}
