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

Q1 = [0.74032,-0.57484,1.7388,1.976,-1.8028,-1.5028]
Q2 = [0.73966,-0.57434,1.7378,1.9765,-1.8021,-1.5028]
Q3 = [0.739,-0.57384,1.7368,1.977,-1.8015,-1.5028]
Q4 = [0.73835,-0.57335,1.7358,1.9775,-1.8008,-1.5028]
Q5 = [0.73769,-0.57285,1.7348,1.978,-1.8001,-1.5028]
Q6 = [0.73704,-0.57235,1.7338,1.9785,-1.7995,-1.5028]
Q7 = [0.73638,-0.57184,1.7328,1.979,-1.7988,-1.5028]
Q8 = [0.73573,-0.57134,1.7318,1.9795,-1.7982,-1.5028]
Q9 = [0.73508,-0.57084,1.7308,1.98,-1.7975,-1.5028]
Q10 = [0.73443,-0.57034,1.7298,1.9805,-1.7969,-1.5028]
Q11 = [0.73378,-0.56984,1.7288,1.9811,-1.7962,-1.5028]
Q12 = [0.73313,-0.56933,1.7277,1.9816,-1.7956,-1.5028]
Q13 = [0.73248,-0.56883,1.7267,1.9821,-1.7949,-1.5028]
Q14 = [0.73183,-0.56832,1.7257,1.9826,-1.7943,-1.5028]
Q15 = [0.73119,-0.56782,1.7247,1.9831,-1.7936,-1.5028]
Q16 = [0.73054,-0.56731,1.7237,1.9836,-1.793,-1.5028]
Q17 = [0.7299,-0.56681,1.7226,1.9841,-1.7923,-1.5028]
Q18 = [0.72925,-0.5663,1.7216,1.9847,-1.7917,-1.5028]
Q19 = [0.72861,-0.5658,1.7206,1.9852,-1.7911,-1.5028]
Q20 = [0.72797,-0.56529,1.7196,1.9857,-1.7904,-1.5028]
Q21 = [0.72732,-0.56478,1.7185,1.9862,-1.7898,-1.5028]
Q22 = [0.72668,-0.56427,1.7175,1.9867,-1.7891,-1.5028]
Q23 = [0.72604,-0.56376,1.7165,1.9873,-1.7885,-1.5028]
Q24 = [0.7254,-0.56325,1.7154,1.9878,-1.7879,-1.5028]
Q25 = [0.72477,-0.56274,1.7144,1.9883,-1.7872,-1.5028]
Q26 = [0.72413,-0.56223,1.7134,1.9888,-1.7866,-1.5028]
Q27 = [0.72349,-0.56172,1.7123,1.9894,-1.7859,-1.5028]
Q28 = [0.72286,-0.56121,1.7113,1.9899,-1.7853,-1.5028]
Q29 = [0.72222,-0.5607,1.7103,1.9904,-1.7847,-1.5028]
Q30 = [0.72159,-0.56019,1.7092,1.991,-1.784,-1.5028]
Q31 = [0.72095,-0.55967,1.7082,1.9915,-1.7834,-1.5028]
Q32 = [0.72032,-0.55916,1.7071,1.992,-1.7828,-1.5028]
Q33 = [0.71969,-0.55865,1.7061,1.9925,-1.7821,-1.5028]
Q34 = [0.71906,-0.55813,1.705,1.9931,-1.7815,-1.5028]
Q35 = [0.71843,-0.55762,1.704,1.9936,-1.7809,-1.5028]
Q36 = [0.7178,-0.5571,1.7029,1.9941,-1.7803,-1.5028]
Q37 = [0.71717,-0.55658,1.7019,1.9947,-1.7796,-1.5028]
Q38 = [0.71654,-0.55607,1.7008,1.9952,-1.779,-1.5028]
Q39 = [0.71592,-0.55555,1.6998,1.9958,-1.7784,-1.5028]
Q40 = [0.71529,-0.55503,1.6987,1.9963,-1.7777,-1.5028]
Q41 = [0.71466,-0.55452,1.6977,1.9968,-1.7771,-1.5028]
Q42 = [0.71404,-0.554,1.6966,1.9974,-1.7765,-1.5028]
Q43 = [0.71342,-0.55348,1.6956,1.9979,-1.7759,-1.5028]
Q44 = [0.71279,-0.55296,1.6945,1.9985,-1.7752,-1.5028]
Q45 = [0.71217,-0.55244,1.6934,1.999,-1.7746,-1.5028]
Q46 = [0.71155,-0.55192,1.6924,1.9995,-1.774,-1.5028]
Q47 = [0.71093,-0.55139,1.6913,2.0001,-1.7734,-1.5028]
Q48 = [0.71031,-0.55087,1.6902,2.0006,-1.7728,-1.5028]
Q49 = [0.70969,-0.55035,1.6892,2.0012,-1.7721,-1.5028]
Q50 = [0.70908,-0.54983,1.6881,2.0017,-1.7715,-1.5028]
Q51 = [0.70846,-0.5493,1.687,2.0023,-1.7709,-1.5028]
Q52 = [0.70784,-0.54878,1.686,2.0028,-1.7703,-1.5028]
Q53 = [0.70723,-0.54826,1.6849,2.0034,-1.7697,-1.5028]
Q54 = [0.70661,-0.54773,1.6838,2.0039,-1.7691,-1.5028]
Q55 = [0.706,-0.5472,1.6827,2.0045,-1.7685,-1.5028]
Q56 = [0.70539,-0.54668,1.6816,2.005,-1.7678,-1.5028]
Q57 = [0.70477,-0.54615,1.6806,2.0056,-1.7672,-1.5028]
Q58 = [0.70416,-0.54562,1.6795,2.0061,-1.7666,-1.5028]
Q59 = [0.70355,-0.5451,1.6784,2.0067,-1.766,-1.5027]
Q60 = [0.70294,-0.54457,1.6773,2.0072,-1.7654,-1.5027]
Q61 = [0.70233,-0.54404,1.6762,2.0078,-1.7648,-1.5027]
Q62 = [0.70172,-0.54351,1.6751,2.0084,-1.7642,-1.5027]
Q63 = [0.70112,-0.54298,1.6741,2.0089,-1.7636,-1.5027]
Q64 = [0.70051,-0.54245,1.673,2.0095,-1.763,-1.5027]
Q65 = [0.6999,-0.54192,1.6719,2.01,-1.7624,-1.5027]
Q66 = [0.6993,-0.54139,1.6708,2.0106,-1.7618,-1.5027]
Q67 = [0.6987,-0.54086,1.6697,2.0112,-1.7611,-1.5027]
Q68 = [0.69809,-0.54032,1.6686,2.0117,-1.7605,-1.5027]
Q69 = [0.69749,-0.53979,1.6675,2.0123,-1.7599,-1.5027]
Q70 = [0.69689,-0.53926,1.6664,2.0129,-1.7593,-1.5027]
Q71 = [0.69629,-0.53872,1.6653,2.0134,-1.7587,-1.5027]
Q72 = [0.69569,-0.53819,1.6642,2.014,-1.7581,-1.5027]
Q73 = [0.69509,-0.53765,1.6631,2.0145,-1.7575,-1.5027]
Q74 = [0.69449,-0.53711,1.662,2.0151,-1.7569,-1.5027]
Q75 = [0.69389,-0.53658,1.6609,2.0157,-1.7563,-1.5027]
Q76 = [0.69329,-0.53604,1.6598,2.0163,-1.7557,-1.5027]
Q77 = [0.6927,-0.5355,1.6587,2.0168,-1.7551,-1.5027]
Q78 = [0.6921,-0.53497,1.6576,2.0174,-1.7546,-1.5027]
Q79 = [0.69151,-0.53443,1.6564,2.018,-1.754,-1.5027]
Q80 = [0.69091,-0.53389,1.6553,2.0185,-1.7534,-1.5027]
Q81 = [0.69032,-0.53335,1.6542,2.0191,-1.7528,-1.5027]
Q82 = [0.68973,-0.53281,1.6531,2.0197,-1.7522,-1.5027]
Q83 = [0.68914,-0.53227,1.652,2.0203,-1.7516,-1.5027]
Q84 = [0.68854,-0.53172,1.6509,2.0209,-1.751,-1.5027]
Q85 = [0.68795,-0.53118,1.6497,2.0214,-1.7504,-1.5027]
Q86 = [0.68736,-0.53064,1.6486,2.022,-1.7498,-1.5027]
Q87 = [0.68678,-0.5301,1.6475,2.0226,-1.7492,-1.5027]
Q88 = [0.68619,-0.52955,1.6464,2.0232,-1.7486,-1.5027]
Q89 = [0.6856,-0.52901,1.6453,2.0238,-1.7481,-1.5027]
Q90 = [0.68502,-0.52846,1.6441,2.0243,-1.7475,-1.5027]
Q91 = [0.68443,-0.52792,1.643,2.0249,-1.7469,-1.5027]
Q92 = [0.68384,-0.52737,1.6419,2.0255,-1.7463,-1.5027]
Q93 = [0.68326,-0.52682,1.6407,2.0261,-1.7457,-1.5027]
Q94 = [0.68268,-0.52628,1.6396,2.0267,-1.7451,-1.5027]
Q95 = [0.6821,-0.52573,1.6385,2.0273,-1.7445,-1.5027]
Q96 = [0.68151,-0.52518,1.6373,2.0278,-1.744,-1.5027]
Q97 = [0.68093,-0.52463,1.6362,2.0284,-1.7434,-1.5027]
Q98 = [0.68035,-0.52408,1.6351,2.029,-1.7428,-1.5027]
Q99 = [0.67977,-0.52353,1.6339,2.0296,-1.7422,-1.5027]
Q100 = [0.67919,-0.52298,1.6328,2.0302,-1.7416,-1.5027]
Q101 = [0.67862,-0.52243,1.6316,2.0308,-1.7411,-1.5027]


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
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        move()
        print "Trajectory finished"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
