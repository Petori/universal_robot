#!/usr/bin/env python
# Simplified by Petori in 2018/3/31
import time
import roslib; roslib.load_manifest('ur_modern_driver')
import rospy
import actionlib
import sys  
import subprocess  
import os
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [0,-1.57,0,-1.57,0,0]
Q2 = [0,-1.57,-0.4,-1.57,0,0]
Q3 = [0,-1.57,-0.8,-1.57,0,0]
Q4 = [0,-1.57,-1.2,-1.57,0,0]
Q5 = [0,-1.57,-1.6,-1.57,0,0]
Q6 = [0,-1.57,-2.0,-1.57,0,0]
Q7 = [0,-1.57,0,-1.57,0,0]
    
client = None

def move():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0,0,-0.4,0,0,0], time_from_start=rospy.Duration(1)),
            JointTrajectoryPoint(positions=Q2, velocities=[0,0,-0.4,0,0,0], time_from_start=rospy.Duration(2)),
            JointTrajectoryPoint(positions=Q3, velocities=[0,0,-0.4,0,0,0], time_from_start=rospy.Duration(3)),
            JointTrajectoryPoint(positions=Q4, velocities=[0,0,-0.4,0,0,0], time_from_start=rospy.Duration(4)),
            JointTrajectoryPoint(positions=Q5, velocities=[0,0,-0.4,0,0,0], time_from_start=rospy.Duration(5)),
            JointTrajectoryPoint(positions=Q6, velocities=[0,0,-0.4,0,0,0], time_from_start=rospy.Duration(6)),
            JointTrajectoryPoint(positions=Q7, velocities=[0,0,0,0,0,0], time_from_start=rospy.Duration(8))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

   
def main():
    global client
    try:
        rospy.init_node("simple_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        child1 = subprocess.Popen('rostopic echo /joint_states >constantspeed2.txt',shell=True)
        move()
        print "Trajectory finished"
        time.sleep(0.5)
        child2 = subprocess.Popen('cp constantspeed2.txt data/constantspeed2.txt',shell=True)
        time.sleep(0.5)
        child3 = subprocess.Popen('rm constantspeed2.txt',shell=True)
        time.sleep(0.5)
        if True:
            child1.kill()
            child2.kill()
            child3.kill()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
