# Usage

---


## Simulation - Gazebo

- ```roslaunch ur_gazebo ur5.launch```


## Simulation - RViz

- ```roslaunch ur_gazebo ur5.launch```
- ```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true```
- ```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```


## Real Hardware - without Moveit!

- ```roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```


## Real Hardware - with Moveit!

- ```roslaunch ur_bringup ur5_bringup.launch robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]```
- ```roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch```
- ```roslaunch ur5_moveit_config moveit_rviz.launch config:=true```




#### Records
2020-8-8
- dmp：最简框架（机器人部分）写好了，准备测试
- 下步考虑如何体现运动平滑
- 下步补充视觉代码

2020-8-7
- 修改CMakeLists.txt，加入对opt/ros/kinetic/include的编译依赖
- 在my_func.h中添加函数实现对ur_arm::PoseMatrix和geometry_msgs::Pose的类型转换。前者是我自定义的位姿，用于正反解，后者是moveit运动控制要用的类型。

2020-7-22
- #include <sys/stat.h>，mkdir需要的头文件
- 创建dmpGrinding和circleGoalDetect

2020-7-3
- add package <ur_modern_driver>
- fix the bug of issues#262 - "RobotStateRT has wrong length:1060"
