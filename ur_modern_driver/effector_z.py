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

Q1 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,4.0402]
Q2 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,4.0402]
Q3 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,3.5166]
Q4 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,3.5166]
Q5 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,2.993]
Q6 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,2.993]
Q7 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,2.4694]
Q8 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,2.4694]
Q9 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,1.9458]
Q10 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,1.9458]
Q11 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,1.4222]
Q12 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,1.4222]
Q13 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,0.89857]
Q14 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,0.89857]
Q15 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,0.37497]
Q16 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,0.37497]
Q17 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-0.14863]
Q18 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-0.14863]
Q19 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-0.67223]
Q20 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-0.67223]
Q21 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-1.1958]
Q22 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-1.1958]
Q23 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-1.7194]
Q24 = [0.25536,-1.5209,-1.8383,-2.2976,-1.3624,-1.7194]


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
            JointTrajectoryPoint(positions=Q10, velocities=[0]*6, time_from_start=rospy.Duration(35)),
            JointTrajectoryPoint(positions=Q11, velocities=[0]*6, time_from_start=rospy.Duration(38)),
            JointTrajectoryPoint(positions=Q12, velocities=[0]*6, time_from_start=rospy.Duration(41)),
            JointTrajectoryPoint(positions=Q13, velocities=[0]*6, time_from_start=rospy.Duration(44)),
            JointTrajectoryPoint(positions=Q14, velocities=[0]*6, time_from_start=rospy.Duration(47)),
            JointTrajectoryPoint(positions=Q15, velocities=[0]*6, time_from_start=rospy.Duration(50)),
            JointTrajectoryPoint(positions=Q16, velocities=[0]*6, time_from_start=rospy.Duration(53)),
            JointTrajectoryPoint(positions=Q17, velocities=[0]*6, time_from_start=rospy.Duration(56)),
            JointTrajectoryPoint(positions=Q18, velocities=[0]*6, time_from_start=rospy.Duration(59)),
            JointTrajectoryPoint(positions=Q19, velocities=[0]*6, time_from_start=rospy.Duration(62)),
            JointTrajectoryPoint(positions=Q20, velocities=[0]*6, time_from_start=rospy.Duration(65)),
            JointTrajectoryPoint(positions=Q21, velocities=[0]*6, time_from_start=rospy.Duration(68)),
            JointTrajectoryPoint(positions=Q22, velocities=[0]*6, time_from_start=rospy.Duration(71)),
            JointTrajectoryPoint(positions=Q23, velocities=[0]*6, time_from_start=rospy.Duration(74)),
            JointTrajectoryPoint(positions=Q24, velocities=[0]*6, time_from_start=rospy.Duration(82))]
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

