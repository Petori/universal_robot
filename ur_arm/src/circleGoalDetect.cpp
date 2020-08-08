// Created by Petori on 2020/7/22
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <fstream>


using namespace std;


int main(int argc, char const *argv[]) {
  /* code */

  // 声明变量和函数
  double circle_pos[3];

  // 开始检测圆


  // 打印结果到窗口供检查
  ofstream fout("/home/petori/data/parameter/cc_pos.txt");
  for(int i=0;i<3;i++)
  {
    fout<<circle_pos[i]<<endl;
  }


  // 将圆心坐标写入文件


  return 0;
}
