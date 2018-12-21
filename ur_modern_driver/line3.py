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

Q1 = [1.2692,-1.3256,2.2291,-2.4773,-1.5418,2.8733]
Q2 = [1.2692,-1.325,2.2293,-2.478,-1.5418,2.8733]
Q3 = [1.2692,-1.3245,2.2295,-2.4788,-1.5418,2.8733]
Q4 = [1.2692,-1.324,2.2297,-2.4795,-1.5418,2.8733]
Q5 = [1.2692,-1.3234,2.2299,-2.4802,-1.5418,2.8733]
Q6 = [1.2692,-1.3229,2.23,-2.4809,-1.5418,2.8733]
Q7 = [1.2692,-1.3224,2.2302,-2.4816,-1.5418,2.8734]
Q8 = [1.2692,-1.3218,2.2304,-2.4823,-1.5418,2.8734]
Q9 = [1.2692,-1.3213,2.2306,-2.4831,-1.5418,2.8734]
Q10 = [1.2692,-1.3207,2.2308,-2.4838,-1.5418,2.8734]
Q11 = [1.2692,-1.3202,2.2309,-2.4845,-1.5418,2.8734]
Q12 = [1.2692,-1.3197,2.2311,-2.4852,-1.5418,2.8734]
Q13 = [1.2692,-1.3191,2.2313,-2.4859,-1.5418,2.8734]
Q14 = [1.2692,-1.3186,2.2315,-2.4866,-1.5418,2.8734]
Q15 = [1.2692,-1.3181,2.2316,-2.4874,-1.5418,2.8734]
Q16 = [1.2692,-1.3175,2.2318,-2.4881,-1.5418,2.8734]
Q17 = [1.2692,-1.317,2.232,-2.4888,-1.5418,2.8734]
Q18 = [1.2692,-1.3164,2.2322,-2.4895,-1.5418,2.8734]
Q19 = [1.2693,-1.3159,2.2323,-2.4902,-1.5418,2.8734]
Q20 = [1.2693,-1.3153,2.2325,-2.4909,-1.5418,2.8734]
Q21 = [1.2693,-1.3148,2.2327,-2.4917,-1.5418,2.8734]
Q22 = [1.2693,-1.3143,2.2329,-2.4924,-1.5418,2.8734]
Q23 = [1.2693,-1.3137,2.233,-2.4931,-1.5418,2.8734]
Q24 = [1.2693,-1.3132,2.2332,-2.4938,-1.5418,2.8734]
Q25 = [1.2693,-1.3126,2.2334,-2.4945,-1.5418,2.8734]
Q26 = [1.2693,-1.3121,2.2336,-2.4952,-1.5418,2.8734]
Q27 = [1.2693,-1.3115,2.2337,-2.496,-1.5418,2.8734]
Q28 = [1.2693,-1.311,2.2339,-2.4967,-1.5418,2.8734]
Q29 = [1.2693,-1.3105,2.2341,-2.4974,-1.5418,2.8734]
Q30 = [1.2693,-1.3099,2.2342,-2.4981,-1.5418,2.8734]
Q31 = [1.2693,-1.3094,2.2344,-2.4988,-1.5418,2.8734]
Q32 = [1.2693,-1.3088,2.2346,-2.4995,-1.5418,2.8734]
Q33 = [1.2693,-1.3083,2.2348,-2.5002,-1.5418,2.8734]
Q34 = [1.2693,-1.3077,2.2349,-2.501,-1.5418,2.8734]
Q35 = [1.2693,-1.3072,2.2351,-2.5017,-1.5418,2.8734]
Q36 = [1.2693,-1.3066,2.2353,-2.5024,-1.5418,2.8734]
Q37 = [1.2693,-1.3061,2.2354,-2.5031,-1.5418,2.8734]
Q38 = [1.2693,-1.3055,2.2356,-2.5038,-1.5418,2.8734]
Q39 = [1.2693,-1.305,2.2358,-2.5045,-1.5418,2.8734]
Q40 = [1.2693,-1.3044,2.2359,-2.5053,-1.5418,2.8734]
Q41 = [1.2693,-1.3039,2.2361,-2.506,-1.5418,2.8734]
Q42 = [1.2693,-1.3033,2.2363,-2.5067,-1.5418,2.8734]
Q43 = [1.2693,-1.3028,2.2364,-2.5074,-1.5418,2.8734]
Q44 = [1.2693,-1.3022,2.2366,-2.5081,-1.5418,2.8735]
Q45 = [1.2693,-1.3017,2.2368,-2.5088,-1.5418,2.8735]
Q46 = [1.2693,-1.3011,2.2369,-2.5096,-1.5418,2.8735]
Q47 = [1.2693,-1.3006,2.2371,-2.5103,-1.5418,2.8735]
Q48 = [1.2693,-1.3,2.2373,-2.511,-1.5418,2.8735]
Q49 = [1.2693,-1.2995,2.2374,-2.5117,-1.5418,2.8735]
Q50 = [1.2693,-1.2989,2.2376,-2.5124,-1.5418,2.8735]
Q51 = [1.2693,-1.2984,2.2378,-2.5131,-1.5418,2.8735]
Q52 = [1.2693,-1.2978,2.2379,-2.5139,-1.5418,2.8735]
Q53 = [1.2693,-1.2973,2.2381,-2.5146,-1.5418,2.8735]
Q54 = [1.2693,-1.2967,2.2382,-2.5153,-1.5418,2.8735]
Q55 = [1.2694,-1.2962,2.2384,-2.516,-1.5418,2.8735]
Q56 = [1.2694,-1.2956,2.2386,-2.5167,-1.5418,2.8735]
Q57 = [1.2694,-1.295,2.2387,-2.5174,-1.5418,2.8735]
Q58 = [1.2694,-1.2945,2.2389,-2.5182,-1.5418,2.8735]
Q59 = [1.2694,-1.2939,2.239,-2.5189,-1.5418,2.8735]
Q60 = [1.2694,-1.2934,2.2392,-2.5196,-1.5418,2.8735]
Q61 = [1.2694,-1.2928,2.2394,-2.5203,-1.5418,2.8735]
Q62 = [1.2694,-1.2923,2.2395,-2.521,-1.5418,2.8735]
Q63 = [1.2694,-1.2917,2.2397,-2.5217,-1.5418,2.8735]
Q64 = [1.2694,-1.2912,2.2398,-2.5225,-1.5418,2.8735]
Q65 = [1.2694,-1.2906,2.24,-2.5232,-1.5418,2.8735]
Q66 = [1.2694,-1.29,2.2402,-2.5239,-1.5418,2.8735]
Q67 = [1.2694,-1.2895,2.2403,-2.5246,-1.5418,2.8735]
Q68 = [1.2694,-1.2889,2.2405,-2.5253,-1.5418,2.8735]
Q69 = [1.2694,-1.2884,2.2406,-2.526,-1.5418,2.8735]
Q70 = [1.2694,-1.2878,2.2408,-2.5267,-1.5418,2.8735]
Q71 = [1.2694,-1.2872,2.2409,-2.5275,-1.5418,2.8735]
Q72 = [1.2694,-1.2867,2.2411,-2.5282,-1.5418,2.8735]
Q73 = [1.2694,-1.2861,2.2412,-2.5289,-1.5418,2.8735]
Q74 = [1.2694,-1.2856,2.2414,-2.5296,-1.5418,2.8735]
Q75 = [1.2694,-1.285,2.2416,-2.5303,-1.5418,2.8735]
Q76 = [1.2694,-1.2844,2.2417,-2.531,-1.5418,2.8735]
Q77 = [1.2694,-1.2839,2.2419,-2.5318,-1.5418,2.8735]
Q78 = [1.2694,-1.2833,2.242,-2.5325,-1.5418,2.8735]
Q79 = [1.2694,-1.2827,2.2422,-2.5332,-1.5418,2.8735]
Q80 = [1.2694,-1.2822,2.2423,-2.5339,-1.5418,2.8736]
Q81 = [1.2694,-1.2816,2.2425,-2.5346,-1.5418,2.8736]
Q82 = [1.2694,-1.2811,2.2426,-2.5353,-1.5418,2.8736]
Q83 = [1.2694,-1.2805,2.2428,-2.5361,-1.5418,2.8736]
Q84 = [1.2694,-1.2799,2.2429,-2.5368,-1.5418,2.8736]
Q85 = [1.2694,-1.2794,2.2431,-2.5375,-1.5418,2.8736]
Q86 = [1.2694,-1.2788,2.2432,-2.5382,-1.5418,2.8736]
Q87 = [1.2694,-1.2782,2.2434,-2.5389,-1.5418,2.8736]
Q88 = [1.2694,-1.2777,2.2435,-2.5396,-1.5418,2.8736]
Q89 = [1.2694,-1.2771,2.2437,-2.5403,-1.5418,2.8736]
Q90 = [1.2694,-1.2765,2.2438,-2.5411,-1.5418,2.8736]
Q91 = [1.2694,-1.276,2.244,-2.5418,-1.5418,2.8736]
Q92 = [1.2695,-1.2754,2.2441,-2.5425,-1.5418,2.8736]
Q93 = [1.2695,-1.2748,2.2443,-2.5432,-1.5418,2.8736]
Q94 = [1.2695,-1.2743,2.2444,-2.5439,-1.5418,2.8736]
Q95 = [1.2695,-1.2737,2.2445,-2.5446,-1.5418,2.8736]
Q96 = [1.2695,-1.2731,2.2447,-2.5453,-1.5418,2.8736]
Q97 = [1.2695,-1.2725,2.2448,-2.5461,-1.5418,2.8736]
Q98 = [1.2695,-1.272,2.245,-2.5468,-1.5418,2.8736]
Q99 = [1.2695,-1.2714,2.2451,-2.5475,-1.5418,2.8736]
Q100 = [1.2695,-1.2708,2.2453,-2.5482,-1.5418,2.8736]
Q101 = [1.2695,-1.2703,2.2454,-2.5489,-1.5418,2.8736]


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
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(5.1)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(5.2)),
            JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(5.3)),
            JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(5.4)),
            JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(5.5)),
            JointTrajectoryPoint(positions=Q7, velocities=[0]*6, time_from_start=rospy.Duration(5.6)),
            JointTrajectoryPoint(positions=Q8, velocities=[0]*6, time_from_start=rospy.Duration(5.7)),
            JointTrajectoryPoint(positions=Q9, velocities=[0]*6, time_from_start=rospy.Duration(5.8)),
            JointTrajectoryPoint(positions=Q10, velocities=[0]*6, time_from_start=rospy.Duration(5.9)),
            JointTrajectoryPoint(positions=Q11, velocities=[0]*6, time_from_start=rospy.Duration(6)),
            JointTrajectoryPoint(positions=Q12, velocities=[0]*6, time_from_start=rospy.Duration(6.1)),
            JointTrajectoryPoint(positions=Q13, velocities=[0]*6, time_from_start=rospy.Duration(6.2)),
            JointTrajectoryPoint(positions=Q14, velocities=[0]*6, time_from_start=rospy.Duration(6.3)),
            JointTrajectoryPoint(positions=Q15, velocities=[0]*6, time_from_start=rospy.Duration(6.4)),
            JointTrajectoryPoint(positions=Q16, velocities=[0]*6, time_from_start=rospy.Duration(6.5)),
            JointTrajectoryPoint(positions=Q17, velocities=[0]*6, time_from_start=rospy.Duration(6.6)),
            JointTrajectoryPoint(positions=Q18, velocities=[0]*6, time_from_start=rospy.Duration(6.7)),
            JointTrajectoryPoint(positions=Q19, velocities=[0]*6, time_from_start=rospy.Duration(6.8)),
            JointTrajectoryPoint(positions=Q20, velocities=[0]*6, time_from_start=rospy.Duration(6.9)),
            JointTrajectoryPoint(positions=Q21, velocities=[0]*6, time_from_start=rospy.Duration(7)),
            JointTrajectoryPoint(positions=Q22, velocities=[0]*6, time_from_start=rospy.Duration(7.1)),
            JointTrajectoryPoint(positions=Q23, velocities=[0]*6, time_from_start=rospy.Duration(7.2)),
            JointTrajectoryPoint(positions=Q24, velocities=[0]*6, time_from_start=rospy.Duration(7.3)),
            JointTrajectoryPoint(positions=Q25, velocities=[0]*6, time_from_start=rospy.Duration(7.4)),
            JointTrajectoryPoint(positions=Q26, velocities=[0]*6, time_from_start=rospy.Duration(7.5)),
            JointTrajectoryPoint(positions=Q27, velocities=[0]*6, time_from_start=rospy.Duration(7.6)),
            JointTrajectoryPoint(positions=Q28, velocities=[0]*6, time_from_start=rospy.Duration(7.7)),
            JointTrajectoryPoint(positions=Q29, velocities=[0]*6, time_from_start=rospy.Duration(7.8)),
            JointTrajectoryPoint(positions=Q30, velocities=[0]*6, time_from_start=rospy.Duration(7.9)),
            JointTrajectoryPoint(positions=Q31, velocities=[0]*6, time_from_start=rospy.Duration(8)),
            JointTrajectoryPoint(positions=Q32, velocities=[0]*6, time_from_start=rospy.Duration(8.1)),
            JointTrajectoryPoint(positions=Q33, velocities=[0]*6, time_from_start=rospy.Duration(8.2)),
            JointTrajectoryPoint(positions=Q34, velocities=[0]*6, time_from_start=rospy.Duration(8.3)),
            JointTrajectoryPoint(positions=Q35, velocities=[0]*6, time_from_start=rospy.Duration(8.4)),
            JointTrajectoryPoint(positions=Q36, velocities=[0]*6, time_from_start=rospy.Duration(8.5)),
            JointTrajectoryPoint(positions=Q37, velocities=[0]*6, time_from_start=rospy.Duration(8.6)),
            JointTrajectoryPoint(positions=Q38, velocities=[0]*6, time_from_start=rospy.Duration(8.7)),
            JointTrajectoryPoint(positions=Q39, velocities=[0]*6, time_from_start=rospy.Duration(8.8)),
            JointTrajectoryPoint(positions=Q40, velocities=[0]*6, time_from_start=rospy.Duration(8.9)),
            JointTrajectoryPoint(positions=Q41, velocities=[0]*6, time_from_start=rospy.Duration(9)),
            JointTrajectoryPoint(positions=Q42, velocities=[0]*6, time_from_start=rospy.Duration(9.1)),
            JointTrajectoryPoint(positions=Q43, velocities=[0]*6, time_from_start=rospy.Duration(9.2)),
            JointTrajectoryPoint(positions=Q44, velocities=[0]*6, time_from_start=rospy.Duration(9.3)),
            JointTrajectoryPoint(positions=Q45, velocities=[0]*6, time_from_start=rospy.Duration(9.4)),
            JointTrajectoryPoint(positions=Q46, velocities=[0]*6, time_from_start=rospy.Duration(9.5)),
            JointTrajectoryPoint(positions=Q47, velocities=[0]*6, time_from_start=rospy.Duration(9.6)),
            JointTrajectoryPoint(positions=Q48, velocities=[0]*6, time_from_start=rospy.Duration(9.7)),
            JointTrajectoryPoint(positions=Q49, velocities=[0]*6, time_from_start=rospy.Duration(9.8)),
            JointTrajectoryPoint(positions=Q50, velocities=[0]*6, time_from_start=rospy.Duration(9.9)),
            JointTrajectoryPoint(positions=Q51, velocities=[0]*6, time_from_start=rospy.Duration(10)),
            JointTrajectoryPoint(positions=Q52, velocities=[0]*6, time_from_start=rospy.Duration(10.1)),
            JointTrajectoryPoint(positions=Q53, velocities=[0]*6, time_from_start=rospy.Duration(10.2)),
            JointTrajectoryPoint(positions=Q54, velocities=[0]*6, time_from_start=rospy.Duration(10.3)),
            JointTrajectoryPoint(positions=Q55, velocities=[0]*6, time_from_start=rospy.Duration(10.4)),
            JointTrajectoryPoint(positions=Q56, velocities=[0]*6, time_from_start=rospy.Duration(10.5)),
            JointTrajectoryPoint(positions=Q57, velocities=[0]*6, time_from_start=rospy.Duration(10.6)),
            JointTrajectoryPoint(positions=Q58, velocities=[0]*6, time_from_start=rospy.Duration(10.7)),
            JointTrajectoryPoint(positions=Q59, velocities=[0]*6, time_from_start=rospy.Duration(10.8)),
            JointTrajectoryPoint(positions=Q60, velocities=[0]*6, time_from_start=rospy.Duration(10.9)),
            JointTrajectoryPoint(positions=Q61, velocities=[0]*6, time_from_start=rospy.Duration(11)),
            JointTrajectoryPoint(positions=Q62, velocities=[0]*6, time_from_start=rospy.Duration(11.1)),
            JointTrajectoryPoint(positions=Q63, velocities=[0]*6, time_from_start=rospy.Duration(11.2)),
            JointTrajectoryPoint(positions=Q64, velocities=[0]*6, time_from_start=rospy.Duration(11.3)),
            JointTrajectoryPoint(positions=Q65, velocities=[0]*6, time_from_start=rospy.Duration(11.4)),
            JointTrajectoryPoint(positions=Q66, velocities=[0]*6, time_from_start=rospy.Duration(11.5)),
            JointTrajectoryPoint(positions=Q67, velocities=[0]*6, time_from_start=rospy.Duration(11.6)),
            JointTrajectoryPoint(positions=Q68, velocities=[0]*6, time_from_start=rospy.Duration(11.7)),
            JointTrajectoryPoint(positions=Q69, velocities=[0]*6, time_from_start=rospy.Duration(11.8)),
            JointTrajectoryPoint(positions=Q70, velocities=[0]*6, time_from_start=rospy.Duration(11.9)),
            JointTrajectoryPoint(positions=Q71, velocities=[0]*6, time_from_start=rospy.Duration(12)),
            JointTrajectoryPoint(positions=Q72, velocities=[0]*6, time_from_start=rospy.Duration(12.1)),
            JointTrajectoryPoint(positions=Q73, velocities=[0]*6, time_from_start=rospy.Duration(12.2)),
            JointTrajectoryPoint(positions=Q74, velocities=[0]*6, time_from_start=rospy.Duration(12.3)),
            JointTrajectoryPoint(positions=Q75, velocities=[0]*6, time_from_start=rospy.Duration(12.4)),
            JointTrajectoryPoint(positions=Q76, velocities=[0]*6, time_from_start=rospy.Duration(12.5)),
            JointTrajectoryPoint(positions=Q77, velocities=[0]*6, time_from_start=rospy.Duration(12.6)),
            JointTrajectoryPoint(positions=Q78, velocities=[0]*6, time_from_start=rospy.Duration(12.7)),
            JointTrajectoryPoint(positions=Q79, velocities=[0]*6, time_from_start=rospy.Duration(12.8)),
            JointTrajectoryPoint(positions=Q80, velocities=[0]*6, time_from_start=rospy.Duration(12.9)),
            JointTrajectoryPoint(positions=Q81, velocities=[0]*6, time_from_start=rospy.Duration(13)),
            JointTrajectoryPoint(positions=Q82, velocities=[0]*6, time_from_start=rospy.Duration(13.1)),
            JointTrajectoryPoint(positions=Q83, velocities=[0]*6, time_from_start=rospy.Duration(13.2)),
            JointTrajectoryPoint(positions=Q84, velocities=[0]*6, time_from_start=rospy.Duration(13.3)),
            JointTrajectoryPoint(positions=Q85, velocities=[0]*6, time_from_start=rospy.Duration(13.4)),
            JointTrajectoryPoint(positions=Q86, velocities=[0]*6, time_from_start=rospy.Duration(13.5)),
            JointTrajectoryPoint(positions=Q87, velocities=[0]*6, time_from_start=rospy.Duration(13.6)),
            JointTrajectoryPoint(positions=Q88, velocities=[0]*6, time_from_start=rospy.Duration(13.7)),
            JointTrajectoryPoint(positions=Q89, velocities=[0]*6, time_from_start=rospy.Duration(13.8)),
            JointTrajectoryPoint(positions=Q90, velocities=[0]*6, time_from_start=rospy.Duration(13.9)),
            JointTrajectoryPoint(positions=Q91, velocities=[0]*6, time_from_start=rospy.Duration(14)),
            JointTrajectoryPoint(positions=Q92, velocities=[0]*6, time_from_start=rospy.Duration(14.1)),
            JointTrajectoryPoint(positions=Q93, velocities=[0]*6, time_from_start=rospy.Duration(14.2)),
            JointTrajectoryPoint(positions=Q94, velocities=[0]*6, time_from_start=rospy.Duration(14.3)),
            JointTrajectoryPoint(positions=Q95, velocities=[0]*6, time_from_start=rospy.Duration(14.4)),
            JointTrajectoryPoint(positions=Q96, velocities=[0]*6, time_from_start=rospy.Duration(14.5)),
            JointTrajectoryPoint(positions=Q97, velocities=[0]*6, time_from_start=rospy.Duration(14.6)),
            JointTrajectoryPoint(positions=Q98, velocities=[0]*6, time_from_start=rospy.Duration(14.7)),
            JointTrajectoryPoint(positions=Q99, velocities=[0]*6, time_from_start=rospy.Duration(14.8)),
            JointTrajectoryPoint(positions=Q100, velocities=[0]*6, time_from_start=rospy.Duration(14.9)),
            JointTrajectoryPoint(positions=Q101, velocities=[0]*6, time_from_start=rospy.Duration(15))]
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
