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
from std_msgs.msg import Int8

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [1.2467,-1.3889,2.3127,-2.4967,-1.5415,2.8503]
Q2 = [1.2467,-1.3883,2.3128,-2.4974,-1.5415,2.8503]
Q3 = [1.2467,-1.3878,2.3129,-2.498,-1.5415,2.8503]
Q4 = [1.2467,-1.3872,2.313,-2.4987,-1.5415,2.8503]
Q5 = [1.2467,-1.3866,2.313,-2.4993,-1.5415,2.8503]
Q6 = [1.2467,-1.386,2.3131,-2.5,-1.5415,2.8503]
Q7 = [1.2467,-1.3854,2.3132,-2.5006,-1.5415,2.8503]
Q8 = [1.2467,-1.3848,2.3133,-2.5013,-1.5415,2.8503]
Q9 = [1.2467,-1.3843,2.3133,-2.502,-1.5415,2.8503]
Q10 = [1.2467,-1.3837,2.3134,-2.5026,-1.5415,2.8503]
Q11 = [1.2467,-1.3831,2.3135,-2.5033,-1.5415,2.8503]
Q12 = [1.2467,-1.3825,2.3136,-2.5039,-1.5415,2.8503]
Q13 = [1.2467,-1.3819,2.3136,-2.5046,-1.5415,2.8503]
Q14 = [1.2467,-1.3813,2.3137,-2.5053,-1.5415,2.8503]
Q15 = [1.2467,-1.3807,2.3138,-2.5059,-1.5415,2.8503]
Q16 = [1.2467,-1.3802,2.3138,-2.5066,-1.5415,2.8503]
Q17 = [1.2467,-1.3796,2.3139,-2.5072,-1.5415,2.8503]
Q18 = [1.2467,-1.379,2.314,-2.5079,-1.5415,2.8503]
Q19 = [1.2467,-1.3784,2.3141,-2.5085,-1.5415,2.8503]
Q20 = [1.2467,-1.3778,2.3141,-2.5092,-1.5415,2.8503]
Q21 = [1.2467,-1.3772,2.3142,-2.5099,-1.5415,2.8503]
Q22 = [1.2467,-1.3766,2.3143,-2.5105,-1.5415,2.8503]
Q23 = [1.2467,-1.3761,2.3143,-2.5112,-1.5415,2.8503]
Q24 = [1.2467,-1.3755,2.3144,-2.5118,-1.5415,2.8503]
Q25 = [1.2467,-1.3749,2.3145,-2.5125,-1.5415,2.8503]
Q26 = [1.2467,-1.3743,2.3145,-2.5131,-1.5415,2.8503]
Q27 = [1.2467,-1.3737,2.3146,-2.5138,-1.5415,2.8503]
Q28 = [1.2467,-1.3731,2.3147,-2.5144,-1.5415,2.8503]
Q29 = [1.2467,-1.3725,2.3147,-2.5151,-1.5415,2.8503]
Q30 = [1.2467,-1.3719,2.3148,-2.5158,-1.5415,2.8503]
Q31 = [1.2467,-1.3713,2.3149,-2.5164,-1.5415,2.8503]
Q32 = [1.2467,-1.3708,2.3149,-2.5171,-1.5415,2.8503]
Q33 = [1.2467,-1.3702,2.315,-2.5177,-1.5415,2.8503]
Q34 = [1.2467,-1.3696,2.3151,-2.5184,-1.5415,2.8503]
Q35 = [1.2467,-1.369,2.3151,-2.519,-1.5415,2.8503]
Q36 = [1.2467,-1.3684,2.3152,-2.5197,-1.5415,2.8503]
Q37 = [1.2467,-1.3678,2.3153,-2.5203,-1.5415,2.8503]
Q38 = [1.2467,-1.3672,2.3153,-2.521,-1.5415,2.8503]
Q39 = [1.2467,-1.3666,2.3154,-2.5217,-1.5415,2.8503]
Q40 = [1.2467,-1.366,2.3154,-2.5223,-1.5415,2.8503]
Q41 = [1.2467,-1.3654,2.3155,-2.523,-1.5415,2.8503]
Q42 = [1.2467,-1.3648,2.3156,-2.5236,-1.5415,2.8503]
Q43 = [1.2467,-1.3642,2.3156,-2.5243,-1.5415,2.8503]
Q44 = [1.2467,-1.3637,2.3157,-2.5249,-1.5415,2.8503]
Q45 = [1.2467,-1.3631,2.3158,-2.5256,-1.5415,2.8503]
Q46 = [1.2467,-1.3625,2.3158,-2.5262,-1.5415,2.8503]
Q47 = [1.2467,-1.3619,2.3159,-2.5269,-1.5415,2.8503]
Q48 = [1.2467,-1.3613,2.3159,-2.5275,-1.5415,2.8503]
Q49 = [1.2467,-1.3607,2.316,-2.5282,-1.5415,2.8503]
Q50 = [1.2467,-1.3601,2.3161,-2.5289,-1.5415,2.8503]
Q51 = [1.2467,-1.3595,2.3161,-2.5295,-1.5415,2.8503]
Q52 = [1.2467,-1.3589,2.3162,-2.5302,-1.5415,2.8504]
Q53 = [1.2467,-1.3583,2.3162,-2.5308,-1.5415,2.8504]
Q54 = [1.2467,-1.3577,2.3163,-2.5315,-1.5415,2.8504]
Q55 = [1.2467,-1.3571,2.3164,-2.5321,-1.5415,2.8504]
Q56 = [1.2467,-1.3565,2.3164,-2.5328,-1.5415,2.8504]
Q57 = [1.2467,-1.3559,2.3165,-2.5334,-1.5415,2.8504]
Q58 = [1.2467,-1.3553,2.3165,-2.5341,-1.5415,2.8504]
Q59 = [1.2467,-1.3547,2.3166,-2.5347,-1.5415,2.8504]
Q60 = [1.2468,-1.3541,2.3166,-2.5354,-1.5415,2.8504]
Q61 = [1.2468,-1.3536,2.3167,-2.536,-1.5415,2.8504]
Q62 = [1.2468,-1.353,2.3168,-2.5367,-1.5415,2.8504]
Q63 = [1.2468,-1.3524,2.3168,-2.5373,-1.5415,2.8504]
Q64 = [1.2468,-1.3518,2.3169,-2.538,-1.5415,2.8504]
Q65 = [1.2468,-1.3512,2.3169,-2.5386,-1.5415,2.8504]
Q66 = [1.2468,-1.3506,2.317,-2.5393,-1.5415,2.8504]
Q67 = [1.2468,-1.35,2.317,-2.5399,-1.5415,2.8504]
Q68 = [1.2468,-1.3494,2.3171,-2.5406,-1.5415,2.8504]
Q69 = [1.2468,-1.3488,2.3171,-2.5412,-1.5415,2.8504]
Q70 = [1.2468,-1.3482,2.3172,-2.5419,-1.5415,2.8504]
Q71 = [1.2468,-1.3476,2.3172,-2.5425,-1.5415,2.8504]
Q72 = [1.2468,-1.347,2.3173,-2.5432,-1.5415,2.8504]
Q73 = [1.2468,-1.3464,2.3173,-2.5438,-1.5415,2.8504]
Q74 = [1.2468,-1.3458,2.3174,-2.5445,-1.5415,2.8504]
Q75 = [1.2468,-1.3452,2.3174,-2.5452,-1.5415,2.8504]
Q76 = [1.2468,-1.3446,2.3175,-2.5458,-1.5415,2.8504]
Q77 = [1.2468,-1.344,2.3175,-2.5465,-1.5415,2.8504]
Q78 = [1.2468,-1.3434,2.3176,-2.5471,-1.5415,2.8504]
Q79 = [1.2468,-1.3428,2.3176,-2.5477,-1.5415,2.8504]
Q80 = [1.2468,-1.3422,2.3177,-2.5484,-1.5415,2.8504]
Q81 = [1.2468,-1.3416,2.3177,-2.549,-1.5415,2.8504]
Q82 = [1.2468,-1.341,2.3178,-2.5497,-1.5415,2.8504]
Q83 = [1.2468,-1.3404,2.3178,-2.5503,-1.5415,2.8504]
Q84 = [1.2468,-1.3398,2.3179,-2.551,-1.5415,2.8504]
Q85 = [1.2468,-1.3392,2.3179,-2.5516,-1.5415,2.8504]
Q86 = [1.2468,-1.3386,2.318,-2.5523,-1.5415,2.8504]
Q87 = [1.2468,-1.338,2.318,-2.5529,-1.5415,2.8504]
Q88 = [1.2468,-1.3374,2.3181,-2.5536,-1.5415,2.8504]
Q89 = [1.2468,-1.3368,2.3181,-2.5542,-1.5415,2.8504]
Q90 = [1.2468,-1.3362,2.3182,-2.5549,-1.5415,2.8504]
Q91 = [1.2468,-1.3356,2.3182,-2.5555,-1.5415,2.8504]
Q92 = [1.2468,-1.335,2.3183,-2.5562,-1.5415,2.8504]
Q93 = [1.2468,-1.3344,2.3183,-2.5568,-1.5415,2.8504]
Q94 = [1.2468,-1.3338,2.3184,-2.5575,-1.5415,2.8504]
Q95 = [1.2468,-1.3332,2.3184,-2.5581,-1.5415,2.8504]
Q96 = [1.2468,-1.3326,2.3184,-2.5588,-1.5415,2.8504]
Q97 = [1.2468,-1.332,2.3185,-2.5594,-1.5415,2.8504]
Q98 = [1.2468,-1.3314,2.3185,-2.5601,-1.5415,2.8504]
Q99 = [1.2468,-1.3308,2.3186,-2.5607,-1.5415,2.8504]
Q100 = [1.2468,-1.3302,2.3186,-2.5614,-1.5415,2.8504]
Q101 = [1.2467,-1.3285,2.3213,-2.575,-1.5409,2.85]
        


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

def talker():
    pub = rospy.Publisher('chatter_test', Int8, queue_size=10)
    rate = rospy.Rate(125) # 10hz
    while not rospy.is_shutdown():
        hello_str = 6
        pub.publish(hello_str)
        rate.sleep()

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
        talker()
        sleep(5.0)
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
