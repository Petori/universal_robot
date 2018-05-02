#!/usr/bin/env python
# Simplified by Petori in 2018/3/31
import time
import roslib; roslib.load_manifest('ur_modern_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

JOINT_NAMES = ['right_shoulder_pan_joint','right_shoulder_lift_joint','right_elbow_joint',
               'right_wrist_1_joint','right_wrist_2_joint','right_wrist_3_joint',
               'left_shoulder_pan_joint','left_shoulder_lift_joint','left_elbow_joint',
               'left_wrist_1_joint','left_wrist_2_joint','left_wrist_3_joint']
#Q1 = [0,-1.57,0,-1.57,0,0,0,-1.57,0,-1.57,0,0]
Q1 = [0.2,-1.37,0.2,-1.37,0.2,0.2,0.2,-1.37,0.2,-1.37,0.2,0.2]
Q2 = [0.3,-1.27,0.3,-1.27,0.3,0.3,0.3,-1.27,0.3,-1.27,0.3,0.3]
Q3 = [0.4,-1.17,0.4,-1.17,0.4,0.4,0.4,-1.17,0.4,-1.17,0.4,0.4]
Q4 = [0.5,-1.07,0.5,-1.07,0.5,0.5,0.5,-1.07,0.5,-1.07,0.5,0.5]
Q5 = [0.6,-0.97,0.6,-0.97,0.6,0.6,0.6,-0.97,0.6,-0.97,0.6,0.6]
Q6 = [0.7,-0.87,0.7,-0.87,0.7,0.7,0.7,-0.87,0.7,-0.87,0.7,0.7]
Q7 = [0.8,-0.77,0.8,-0.77,0.8,0.8,0.8,-0.77,0.8,-0.77,0.8,0.8]

    
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
            JointTrajectoryPoint(positions=joints_pos, velocities=[0.1]*12, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0.1]*12, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0.1]*12, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0.1]*12, time_from_start=rospy.Duration(4.0)),
            JointTrajectoryPoint(positions=Q4, velocities=[0.1]*12, time_from_start=rospy.Duration(5.0)),
            JointTrajectoryPoint(positions=Q5, velocities=[0.1]*12, time_from_start=rospy.Duration(6.0)),
            JointTrajectoryPoint(positions=Q6, velocities=[0.1]*12, time_from_start=rospy.Duration(7.0)),
            JointTrajectoryPoint(positions=Q7, velocities=[0.1]*12, time_from_start=rospy.Duration(8.0))]

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
        rospy.init_node("uu_simple_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        move()
        print "Trajectory finished"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
