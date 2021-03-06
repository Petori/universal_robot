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
2020-8-18
- 根据circleGoalDetect的三点调整dmpGrinding，完成整体框架
- 增加dmp_pre_grinding.cpp用于生成供模仿的打磨路径,也可以用于测试相机测量的偏差值
- dmpGrinding.cpp调试完毕，偏差值以后根据打磨情况再做调整
- 明日工作：把代码里关于实物和仿真的部分用宏区分开来

2020-8-17
- circleGoalDetect测试完成，精度很差，约有10cm
- minitest的更改是为了测试话题读取

2020-8-16
- 编写circleGoalDetect.cpp的大致框架

2020-8-12
- 把ur_arm/src的四个文件还原到初始状态，丢失的是为了打磨实验做的更改（速度、加速度等

2020-8-11
- 新建消息类型ur_arm/cartesianState
- 添加函数rot2rpy
- 新建函数jointVel_2_cartesianVel（已通过测试
- 完成代码整体逻辑，等待测试
- (1) 实时运行的版本——对控制器要求高
- (2) 发送预先规划的完整轨迹的版本——测试成功

2020-8-10
- 基于moveit的框架已经控制并调试完毕，但是其无法控制到达路径点的速度和时刻
- 接下来改用joint_trajectory_controller控制（关节空间层面），本版本(dmpGrinding.cpp)上传用于日后参考
- 完成调用actionlib的client进行机器人运动控制的测试
- 接下来需要处理关节空间到笛卡尔空间的速度转换问题

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
