#!/usr/bin/env python
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

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [-0.58574,-1.2836,-1.3817,-2.9191,-2.0407,-1.2072]
Q2 = [-0.58574,-1.2836,-1.3817,-2.9191,-2.0407,-1.2072]
Q3 = [1.3113,-1.1827,-1.5567,-2.3243,-0.65715,-2.7082]
Q4 = [1.3113,-1.1827,-1.5567,-2.3243,-0.65715,-2.7082]
Q5 = [0.50267,-2.4229,-1.2356,-1.9505,-1.1651,-1.8763]
Q6 = [0.50267,-2.4229,-1.2356,-1.9505,-1.1651,-1.8763]
Q7 = [-0.20263,-2.4282,-1.2158,-2.0186,-1.7364,-1.4535]
Q8 = [-0.20263,-2.4282,-1.2158,-2.0186,-1.7364,-1.4535]
Q9 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-1.7194]
Q10 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-1.7194]


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
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(5)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(11)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(14)),
            JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(17)),
            JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(20)),
            JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(23)),
            JointTrajectoryPoint(positions=Q7, velocities=[0]*6, time_from_start=rospy.Duration(26)),
            JointTrajectoryPoint(positions=Q8, velocities=[0]*6, time_from_start=rospy.Duration(29)),
            JointTrajectoryPoint(positions=Q9, velocities=[0]*6, time_from_start=rospy.Duration(32)),
            JointTrajectoryPoint(positions=Q10, velocities=[0]*6, time_from_start=rospy.Duration(40))]
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
        move()
        print "Trajectory finished"
        time.sleep(0.5)
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()

