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
