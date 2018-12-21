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

Q1 = [1.2563,-1.3635,2.2785,-2.4885,-1.5416,2.8602]
Q2 = [1.2563,-1.3629,2.2787,-2.4892,-1.5416,2.8602]
Q3 = [1.2563,-1.3623,2.2789,-2.49,-1.5416,2.8602]
Q4 = [1.2563,-1.3618,2.2791,-2.4907,-1.5416,2.8602]
Q5 = [1.2563,-1.3612,2.2793,-2.4915,-1.5416,2.8602]
Q6 = [1.2563,-1.3606,2.2795,-2.4922,-1.5416,2.8603]
Q7 = [1.2563,-1.3601,2.2797,-2.493,-1.5416,2.8603]
Q8 = [1.2563,-1.3595,2.2798,-2.4937,-1.5416,2.8603]
Q9 = [1.2563,-1.3589,2.28,-2.4945,-1.5416,2.8603]
Q10 = [1.2563,-1.3584,2.2802,-2.4952,-1.5416,2.8603]
Q11 = [1.2563,-1.3578,2.2804,-2.496,-1.5416,2.8603]
Q12 = [1.2563,-1.3573,2.2806,-2.4967,-1.5416,2.8603]
Q13 = [1.2563,-1.3567,2.2808,-2.4975,-1.5416,2.8603]
Q14 = [1.2563,-1.3561,2.281,-2.4983,-1.5416,2.8603]
Q15 = [1.2563,-1.3556,2.2811,-2.499,-1.5416,2.8603]
Q16 = [1.2563,-1.355,2.2813,-2.4998,-1.5416,2.8603]
Q17 = [1.2563,-1.3544,2.2815,-2.5005,-1.5416,2.8603]
Q18 = [1.2563,-1.3539,2.2817,-2.5013,-1.5416,2.8603]
Q19 = [1.2563,-1.3533,2.2819,-2.502,-1.5416,2.8603]
Q20 = [1.2563,-1.3527,2.2821,-2.5028,-1.5416,2.8603]
Q21 = [1.2563,-1.3521,2.2822,-2.5035,-1.5416,2.8603]
Q22 = [1.2563,-1.3516,2.2824,-2.5043,-1.5416,2.8603]
Q23 = [1.2563,-1.351,2.2826,-2.505,-1.5416,2.8603]
Q24 = [1.2563,-1.3504,2.2828,-2.5058,-1.5416,2.8603]
Q25 = [1.2563,-1.3499,2.283,-2.5065,-1.5416,2.8603]
Q26 = [1.2563,-1.3493,2.2831,-2.5073,-1.5416,2.8603]
Q27 = [1.2563,-1.3487,2.2833,-2.508,-1.5416,2.8603]
Q28 = [1.2563,-1.3482,2.2835,-2.5088,-1.5416,2.8603]
Q29 = [1.2563,-1.3476,2.2837,-2.5095,-1.5416,2.8603]
Q30 = [1.2563,-1.347,2.2839,-2.5103,-1.5416,2.8603]
Q31 = [1.2563,-1.3464,2.284,-2.511,-1.5416,2.8603]
Q32 = [1.2563,-1.3459,2.2842,-2.5118,-1.5416,2.8603]
Q33 = [1.2563,-1.3453,2.2844,-2.5125,-1.5416,2.8603]
Q34 = [1.2563,-1.3447,2.2846,-2.5133,-1.5416,2.8603]
Q35 = [1.2563,-1.3441,2.2848,-2.514,-1.5416,2.8603]
Q36 = [1.2563,-1.3436,2.2849,-2.5148,-1.5416,2.8603]
Q37 = [1.2563,-1.343,2.2851,-2.5155,-1.5416,2.8603]
Q38 = [1.2563,-1.3424,2.2853,-2.5163,-1.5416,2.8603]
Q39 = [1.2563,-1.3418,2.2855,-2.517,-1.5416,2.8603]
Q40 = [1.2563,-1.3413,2.2856,-2.5178,-1.5416,2.8603]
Q41 = [1.2563,-1.3407,2.2858,-2.5185,-1.5416,2.8603]
Q42 = [1.2563,-1.3401,2.286,-2.5193,-1.5416,2.8603]
Q43 = [1.2563,-1.3395,2.2862,-2.52,-1.5416,2.8603]
Q44 = [1.2563,-1.3389,2.2863,-2.5208,-1.5416,2.8603]
Q45 = [1.2563,-1.3384,2.2865,-2.5215,-1.5416,2.8603]
Q46 = [1.2563,-1.3378,2.2867,-2.5223,-1.5416,2.8603]
Q47 = [1.2563,-1.3372,2.2868,-2.523,-1.5416,2.8603]
Q48 = [1.2563,-1.3366,2.287,-2.5238,-1.5416,2.8603]
Q49 = [1.2563,-1.3361,2.2872,-2.5246,-1.5416,2.8603]
Q50 = [1.2563,-1.3355,2.2874,-2.5253,-1.5416,2.8603]
Q51 = [1.2563,-1.3349,2.2875,-2.5261,-1.5416,2.8603]
Q52 = [1.2563,-1.3343,2.2877,-2.5268,-1.5416,2.8603]
Q53 = [1.2563,-1.3337,2.2879,-2.5276,-1.5416,2.8603]
Q54 = [1.2563,-1.3331,2.288,-2.5283,-1.5416,2.8603]
Q55 = [1.2563,-1.3326,2.2882,-2.5291,-1.5416,2.8603]
Q56 = [1.2563,-1.332,2.2884,-2.5298,-1.5416,2.8603]
Q57 = [1.2563,-1.3314,2.2885,-2.5306,-1.5416,2.8603]
Q58 = [1.2563,-1.3308,2.2887,-2.5313,-1.5416,2.8603]
Q59 = [1.2563,-1.3302,2.2889,-2.5321,-1.5416,2.8603]
Q60 = [1.2563,-1.3296,2.289,-2.5328,-1.5416,2.8603]
Q61 = [1.2563,-1.3291,2.2892,-2.5336,-1.5416,2.8603]
Q62 = [1.2564,-1.3285,2.2894,-2.5343,-1.5416,2.8603]
Q63 = [1.2564,-1.3279,2.2895,-2.5351,-1.5416,2.8603]
Q64 = [1.2564,-1.3273,2.2897,-2.5358,-1.5416,2.8603]
Q65 = [1.2564,-1.3267,2.2899,-2.5366,-1.5416,2.8603]
Q66 = [1.2564,-1.3261,2.29,-2.5373,-1.5416,2.8603]
Q67 = [1.2564,-1.3255,2.2902,-2.5381,-1.5416,2.8603]
Q68 = [1.2564,-1.3249,2.2904,-2.5388,-1.5416,2.8603]
Q69 = [1.2564,-1.3244,2.2905,-2.5396,-1.5416,2.8603]
Q70 = [1.2564,-1.3238,2.2907,-2.5403,-1.5416,2.8603]
Q71 = [1.2564,-1.3232,2.2909,-2.5411,-1.5416,2.8603]
Q72 = [1.2564,-1.3226,2.291,-2.5418,-1.5416,2.8603]
Q73 = [1.2564,-1.322,2.2912,-2.5426,-1.5416,2.8603]
Q74 = [1.2564,-1.3214,2.2913,-2.5433,-1.5416,2.8603]
Q75 = [1.2564,-1.3208,2.2915,-2.5441,-1.5416,2.8603]
Q76 = [1.2564,-1.3202,2.2917,-2.5448,-1.5416,2.8603]
Q77 = [1.2564,-1.3196,2.2918,-2.5456,-1.5416,2.8603]
Q78 = [1.2564,-1.319,2.292,-2.5464,-1.5416,2.8603]
Q79 = [1.2564,-1.3185,2.2921,-2.5471,-1.5416,2.8603]
Q80 = [1.2564,-1.3179,2.2923,-2.5479,-1.5416,2.8603]
Q81 = [1.2564,-1.3173,2.2925,-2.5486,-1.5416,2.8603]
Q82 = [1.2564,-1.3167,2.2926,-2.5494,-1.5416,2.8603]
Q83 = [1.2564,-1.3161,2.2928,-2.5501,-1.5416,2.8603]
Q84 = [1.2564,-1.3155,2.2929,-2.5509,-1.5416,2.8603]
Q85 = [1.2564,-1.3149,2.2931,-2.5516,-1.5416,2.8603]
Q86 = [1.2564,-1.3143,2.2933,-2.5524,-1.5416,2.8603]
Q87 = [1.2564,-1.3137,2.2934,-2.5531,-1.5416,2.8603]
Q88 = [1.2564,-1.3131,2.2936,-2.5539,-1.5416,2.8603]
Q89 = [1.2564,-1.3125,2.2937,-2.5546,-1.5416,2.8603]
Q90 = [1.2564,-1.3119,2.2939,-2.5554,-1.5416,2.8603]
Q91 = [1.2564,-1.3113,2.294,-2.5561,-1.5416,2.8603]
Q92 = [1.2564,-1.3107,2.2942,-2.5569,-1.5416,2.8603]
Q93 = [1.2564,-1.3101,2.2943,-2.5576,-1.5416,2.8603]
Q94 = [1.2564,-1.3095,2.2945,-2.5584,-1.5416,2.8603]
Q95 = [1.2564,-1.3089,2.2946,-2.5591,-1.5416,2.8603]
Q96 = [1.2564,-1.3083,2.2948,-2.5599,-1.5416,2.8603]
Q97 = [1.2564,-1.3077,2.2949,-2.5606,-1.5416,2.8603]
Q98 = [1.2564,-1.3071,2.2951,-2.5614,-1.5416,2.8603]
Q99 = [1.2564,-1.3065,2.2953,-2.5621,-1.5416,2.8603]
Q100 = [1.2564,-1.3059,2.2954,-2.5629,-1.5416,2.8603]
Q101 = [1.2564,-1.3053,2.2956,-2.5636,-1.5416,2.8603]


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
