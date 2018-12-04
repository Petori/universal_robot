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
from std_msgs.msg import Int8
from math import pi

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

Q1 = [1.2291,-1.3865,2.3239,-2.5199,-1.5411,2.8327]
Q2 = [1.2291,-1.386,2.3241,-2.5206,-1.5411,2.8327]
Q3 = [1.2291,-1.3854,2.3242,-2.5213,-1.5411,2.8327]
Q4 = [1.2291,-1.3849,2.3244,-2.522,-1.5411,2.8327]
Q5 = [1.2291,-1.3844,2.3246,-2.5227,-1.5411,2.8327]
Q6 = [1.2291,-1.3838,2.3247,-2.5234,-1.5411,2.8327]
Q7 = [1.2291,-1.3833,2.3249,-2.5241,-1.5411,2.8327]
Q8 = [1.2291,-1.3828,2.3251,-2.5248,-1.5411,2.8327]
Q9 = [1.2291,-1.3823,2.3252,-2.5255,-1.5411,2.8327]
Q10 = [1.2291,-1.3817,2.3254,-2.5262,-1.5411,2.8327]
Q11 = [1.2291,-1.3812,2.3256,-2.5269,-1.5411,2.8327]
Q12 = [1.2291,-1.3807,2.3257,-2.5276,-1.5411,2.8327]
Q13 = [1.2291,-1.3801,2.3259,-2.5283,-1.5411,2.8327]
Q14 = [1.2291,-1.3796,2.3261,-2.529,-1.5411,2.8327]
Q15 = [1.2291,-1.3791,2.3262,-2.5297,-1.5411,2.8327]
Q16 = [1.2291,-1.3785,2.3264,-2.5304,-1.5411,2.8327]
Q17 = [1.2291,-1.378,2.3266,-2.531,-1.5411,2.8327]
Q18 = [1.2291,-1.3775,2.3267,-2.5317,-1.5411,2.8327]
Q19 = [1.2291,-1.3769,2.3269,-2.5324,-1.5411,2.8327]
Q20 = [1.2291,-1.3764,2.3271,-2.5331,-1.5411,2.8327]
Q21 = [1.2291,-1.3759,2.3272,-2.5338,-1.5411,2.8327]
Q22 = [1.2291,-1.3753,2.3274,-2.5345,-1.5411,2.8327]
Q23 = [1.2291,-1.3748,2.3275,-2.5352,-1.5411,2.8327]
Q24 = [1.2291,-1.3743,2.3277,-2.5359,-1.5411,2.8327]
Q25 = [1.2291,-1.3737,2.3279,-2.5366,-1.5411,2.8327]
Q26 = [1.2291,-1.3732,2.328,-2.5373,-1.5411,2.8327]
Q27 = [1.2291,-1.3727,2.3282,-2.538,-1.5411,2.8328]
Q28 = [1.2291,-1.3721,2.3283,-2.5387,-1.5411,2.8328]
Q29 = [1.2291,-1.3716,2.3285,-2.5394,-1.5411,2.8328]
Q30 = [1.2292,-1.371,2.3287,-2.5401,-1.5411,2.8328]
Q31 = [1.2292,-1.3705,2.3288,-2.5408,-1.5411,2.8328]
Q32 = [1.2292,-1.37,2.329,-2.5415,-1.5411,2.8328]
Q33 = [1.2292,-1.3694,2.3291,-2.5422,-1.5411,2.8328]
Q34 = [1.2292,-1.3689,2.3293,-2.5429,-1.5411,2.8328]
Q35 = [1.2292,-1.3684,2.3295,-2.5436,-1.5411,2.8328]
Q36 = [1.2292,-1.3678,2.3296,-2.5443,-1.5411,2.8328]
Q37 = [1.2292,-1.3673,2.3298,-2.545,-1.5411,2.8328]
Q38 = [1.2292,-1.3667,2.3299,-2.5457,-1.5411,2.8328]
Q39 = [1.2292,-1.3662,2.3301,-2.5464,-1.5411,2.8328]
Q40 = [1.2292,-1.3657,2.3302,-2.5471,-1.5411,2.8328]
Q41 = [1.2292,-1.3651,2.3304,-2.5478,-1.5411,2.8328]
Q42 = [1.2292,-1.3646,2.3306,-2.5485,-1.5411,2.8328]
Q43 = [1.2292,-1.364,2.3307,-2.5492,-1.5411,2.8328]
Q44 = [1.2292,-1.3635,2.3309,-2.5499,-1.5411,2.8328]
Q45 = [1.2292,-1.3629,2.331,-2.5506,-1.5411,2.8328]
Q46 = [1.2292,-1.3624,2.3312,-2.5513,-1.5411,2.8328]
Q47 = [1.2292,-1.3619,2.3313,-2.552,-1.5411,2.8328]
Q48 = [1.2292,-1.3613,2.3315,-2.5527,-1.5411,2.8328]
Q49 = [1.2292,-1.3608,2.3316,-2.5534,-1.5411,2.8328]
Q50 = [1.2292,-1.3602,2.3318,-2.5541,-1.5411,2.8328]
Q51 = [1.2292,-1.3597,2.3319,-2.5548,-1.5411,2.8328]
Q52 = [1.2292,-1.3591,2.3321,-2.5554,-1.5411,2.8328]
Q53 = [1.2292,-1.3586,2.3322,-2.5561,-1.5411,2.8328]
Q54 = [1.2292,-1.358,2.3324,-2.5568,-1.5411,2.8328]
Q55 = [1.2292,-1.3575,2.3325,-2.5575,-1.5411,2.8328]
Q56 = [1.2292,-1.357,2.3327,-2.5582,-1.5411,2.8328]
Q57 = [1.2292,-1.3564,2.3329,-2.5589,-1.5411,2.8328]
Q58 = [1.2292,-1.3559,2.333,-2.5596,-1.5411,2.8329]
Q59 = [1.2292,-1.3553,2.3332,-2.5603,-1.5411,2.8329]
Q60 = [1.2292,-1.3548,2.3333,-2.561,-1.5411,2.8329]
Q61 = [1.2293,-1.3542,2.3335,-2.5617,-1.5411,2.8329]
Q62 = [1.2293,-1.3537,2.3336,-2.5624,-1.5411,2.8329]
Q63 = [1.2293,-1.3531,2.3337,-2.5631,-1.5411,2.8329]
Q64 = [1.2293,-1.3526,2.3339,-2.5638,-1.5411,2.8329]
Q65 = [1.2293,-1.352,2.334,-2.5645,-1.5411,2.8329]
Q66 = [1.2293,-1.3515,2.3342,-2.5652,-1.5411,2.8329]
Q67 = [1.2293,-1.3509,2.3343,-2.5659,-1.5411,2.8329]
Q68 = [1.2293,-1.3504,2.3345,-2.5666,-1.5411,2.8329]
Q69 = [1.2293,-1.3498,2.3346,-2.5673,-1.5411,2.8329]
Q70 = [1.2293,-1.3493,2.3348,-2.568,-1.5411,2.8329]
Q71 = [1.2293,-1.3487,2.3349,-2.5687,-1.5411,2.8329]
Q72 = [1.2293,-1.3482,2.3351,-2.5694,-1.5411,2.8329]
Q73 = [1.2293,-1.3476,2.3352,-2.5701,-1.5411,2.8329]
Q74 = [1.2293,-1.3471,2.3354,-2.5708,-1.5411,2.8329]
Q75 = [1.2293,-1.3465,2.3355,-2.5715,-1.5411,2.8329]
Q76 = [1.2293,-1.346,2.3357,-2.5722,-1.5411,2.8329]
Q77 = [1.2293,-1.3454,2.3358,-2.5729,-1.5411,2.8329]
Q78 = [1.2293,-1.3449,2.3359,-2.5736,-1.5411,2.8329]
Q79 = [1.2293,-1.3443,2.3361,-2.5743,-1.5411,2.8329]
Q80 = [1.2293,-1.3437,2.3362,-2.575,-1.5411,2.8329]
Q81 = [1.2293,-1.3432,2.3364,-2.5757,-1.5411,2.8329]
Q82 = [1.2293,-1.3426,2.3365,-2.5764,-1.5411,2.8329]
Q83 = [1.2293,-1.3421,2.3367,-2.5771,-1.5411,2.8329]
Q84 = [1.2293,-1.3415,2.3368,-2.5778,-1.5411,2.8329]
Q85 = [1.2293,-1.341,2.3369,-2.5785,-1.5411,2.8329]
Q86 = [1.2293,-1.3404,2.3371,-2.5792,-1.5411,2.8329]
Q87 = [1.2293,-1.3399,2.3372,-2.5799,-1.5411,2.8329]
Q88 = [1.2293,-1.3393,2.3374,-2.5806,-1.5411,2.8329]
Q89 = [1.2293,-1.3387,2.3375,-2.5813,-1.5411,2.8329]
Q90 = [1.2293,-1.3382,2.3377,-2.582,-1.5411,2.833]
Q91 = [1.2293,-1.3376,2.3378,-2.5827,-1.5411,2.833]
Q92 = [1.2293,-1.3371,2.3379,-2.5834,-1.5411,2.833]
Q93 = [1.2294,-1.3365,2.3381,-2.5841,-1.5411,2.833]
Q94 = [1.2294,-1.336,2.3382,-2.5848,-1.5411,2.833]
Q95 = [1.2294,-1.3354,2.3384,-2.5855,-1.5411,2.833]
Q96 = [1.2294,-1.3348,2.3385,-2.5862,-1.5411,2.833]
Q97 = [1.2294,-1.3343,2.3386,-2.5869,-1.5411,2.833]
Q98 = [1.2294,-1.3337,2.3388,-2.5876,-1.5411,2.833]
Q99 = [1.2294,-1.3332,2.3389,-2.5882,-1.5411,2.833]
Q100 = [1.2294,-1.3326,2.339,-2.5889,-1.5411,2.833]
Q101 = [1.2294,-1.332,2.3392,-2.5896,-1.5411,2.833]


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
    rate = rospy.Rate(125)
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
