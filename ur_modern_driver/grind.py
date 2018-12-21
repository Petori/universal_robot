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

Q1 = [1.7156,-1.4704,2.0983,-2.2012,-1.5705,1.7379]
Q2 = [1.7205,-1.4684,2.096,-2.2009,-1.5705,1.7421]
Q3 = [1.7254,-1.4663,2.0937,-2.2007,-1.5705,1.7463]
Q4 = [1.7303,-1.4643,2.0914,-2.2004,-1.5705,1.7504]
Q5 = [1.7351,-1.4623,2.0891,-2.2001,-1.5706,1.7546]
Q6 = [1.74,-1.4602,2.0867,-2.1998,-1.5706,1.7588]
Q7 = [1.7448,-1.4581,2.0843,-2.1995,-1.5706,1.763]
Q8 = [1.7496,-1.456,2.0819,-2.1991,-1.5706,1.7672]
Q9 = [1.7544,-1.4539,2.0794,-2.1988,-1.5706,1.7714]
Q10 = [1.7592,-1.4518,2.077,-2.1985,-1.5706,1.7756]
Q11 = [1.764,-1.4496,2.0745,-2.1981,-1.5706,1.7798]
Q12 = [1.7687,-1.4475,2.072,-2.1978,-1.5706,1.784]
Q13 = [1.7735,-1.4453,2.0695,-2.1974,-1.5707,1.7882]
Q14 = [1.7782,-1.4431,2.0669,-2.1971,-1.5707,1.7924]
Q15 = [1.7829,-1.4409,2.0643,-2.1967,-1.5707,1.7966]
Q16 = [1.7876,-1.4387,2.0617,-2.1963,-1.5707,1.8008]
Q17 = [1.7923,-1.4365,2.0591,-2.1959,-1.5707,1.805]
Q18 = [1.797,-1.4343,2.0565,-2.1955,-1.5707,1.8092]
Q19 = [1.8016,-1.432,2.0538,-2.1951,-1.5707,1.8135]
Q20 = [1.8063,-1.4297,2.0511,-2.1947,-1.5707,1.8177]
Q21 = [1.8109,-1.4275,2.0484,-2.1943,-1.5707,1.8219]
Q22 = [1.8155,-1.4252,2.0457,-2.1938,-1.5708,1.8261]
Q23 = [1.8201,-1.4228,2.0429,-2.1934,-1.5708,1.8304]
Q24 = [1.8246,-1.4205,2.0401,-2.1929,-1.5708,1.8346]
Q25 = [1.8292,-1.4182,2.0373,-2.1924,-1.5708,1.8388]
Q26 = [1.8338,-1.4158,2.0345,-2.192,-1.5708,1.843]
Q27 = [1.8383,-1.4135,2.0317,-2.1915,-1.5708,1.8473]
Q28 = [1.8428,-1.4111,2.0288,-2.191,-1.5708,1.8515]
Q29 = [1.8473,-1.4087,2.0259,-2.1905,-1.5708,1.8557]
Q30 = [1.8518,-1.4063,2.023,-2.1899,-1.5709,1.8599]
Q31 = [1.8562,-1.4039,2.02,-2.1894,-1.5709,1.8642]
Q32 = [1.8607,-1.4015,2.0171,-2.1889,-1.5709,1.8684]
Q33 = [1.8651,-1.399,2.0141,-2.1883,-1.5709,1.8726]
Q34 = [1.8695,-1.3966,2.0111,-2.1878,-1.5709,1.8768]
Q35 = [1.8739,-1.3941,2.008,-2.1872,-1.5709,1.881]
Q36 = [1.8783,-1.3916,2.005,-2.1866,-1.5709,1.8852]
Q37 = [1.8827,-1.3891,2.0019,-2.186,-1.5709,1.8894]
Q38 = [1.887,-1.3866,1.9988,-2.1854,-1.5709,1.8936]
Q39 = [1.8914,-1.3841,1.9956,-2.1848,-1.5709,1.8978]
Q40 = [1.8957,-1.3816,1.9925,-2.1842,-1.571,1.902]
Q41 = [1.9,-1.379,1.9893,-2.1835,-1.571,1.9062]
Q42 = [1.9043,-1.3765,1.9861,-2.1829,-1.571,1.9104]
Q43 = [1.9085,-1.3739,1.9829,-2.1822,-1.571,1.9146]
Q44 = [1.9128,-1.3713,1.9796,-2.1816,-1.571,1.9188]
Q45 = [1.917,-1.3687,1.9763,-2.1809,-1.571,1.923]
Q46 = [1.9212,-1.3661,1.973,-2.1802,-1.571,1.9271]
Q47 = [1.9254,-1.3635,1.9697,-2.1795,-1.571,1.9313]
Q48 = [1.9296,-1.3609,1.9664,-2.1788,-1.571,1.9355]
Q49 = [1.9338,-1.3582,1.963,-2.178,-1.5711,1.9396]
Q50 = [1.9379,-1.3556,1.9596,-2.1773,-1.5711,1.9438]
Q51 = [1.9421,-1.3529,1.9562,-2.1765,-1.5711,1.9479]
Q52 = [1.9462,-1.3502,1.9527,-2.1758,-1.5711,1.9521]
Q53 = [1.9503,-1.3476,1.9493,-2.175,-1.5711,1.9562]
Q54 = [1.9544,-1.3449,1.9458,-2.1742,-1.5711,1.9603]
Q55 = [1.9584,-1.3421,1.9423,-2.1734,-1.5711,1.9645]
Q56 = [1.9625,-1.3394,1.9387,-2.1726,-1.5711,1.9686]
Q57 = [1.9665,-1.3367,1.9351,-2.1717,-1.5711,1.9727]
Q58 = [1.9705,-1.3339,1.9316,-2.1709,-1.5711,1.9768]
Q59 = [1.9745,-1.3312,1.9279,-2.17,-1.5712,1.9809]
Q60 = [1.9785,-1.3284,1.9243,-2.1692,-1.5712,1.985]
Q61 = [1.9824,-1.3256,1.9206,-2.1683,-1.5712,1.9891]
Q62 = [1.9864,-1.3228,1.917,-2.1674,-1.5712,1.9931]
Q63 = [1.9903,-1.32,1.9132,-2.1665,-1.5712,1.9972]
Q64 = [1.9942,-1.3172,1.9095,-2.1656,-1.5712,2.0012]
Q65 = [1.9981,-1.3144,1.9058,-2.1646,-1.5712,2.0053]
Q66 = [2.002,-1.3115,1.902,-2.1637,-1.5712,2.0093]
Q67 = [2.0058,-1.3087,1.8982,-2.1627,-1.5712,2.0134]
Q68 = [2.0097,-1.3058,1.8943,-2.1618,-1.5712,2.0174]
Q69 = [2.0135,-1.3029,1.8905,-2.1608,-1.5713,2.0214]
Q70 = [2.0173,-1.3001,1.8866,-2.1598,-1.5713,2.0254]
Q71 = [2.0211,-1.2972,1.8827,-2.1588,-1.5713,2.0294]
Q72 = [2.0248,-1.2943,1.8787,-2.1577,-1.5713,2.0334]
Q73 = [2.0286,-1.2913,1.8748,-2.1567,-1.5713,2.0374]
Q74 = [2.0323,-1.2884,1.8708,-2.1556,-1.5713,2.0413]
Q75 = [2.036,-1.2855,1.8668,-2.1546,-1.5713,2.0453]
Q76 = [2.0397,-1.2825,1.8628,-2.1535,-1.5713,2.0492]
Q77 = [2.0434,-1.2796,1.8587,-2.1524,-1.5713,2.0532]
Q78 = [2.0471,-1.2766,1.8546,-2.1513,-1.5713,2.0571]
Q79 = [2.0507,-1.2736,1.8505,-2.1501,-1.5713,2.061]
Q80 = [2.0544,-1.2706,1.8464,-2.149,-1.5714,2.0649]
Q81 = [2.058,-1.2676,1.8422,-2.1478,-1.5714,2.0688]
Q82 = [2.0616,-1.2646,1.8381,-2.1467,-1.5714,2.0727]
Q83 = [2.0651,-1.2616,1.8339,-2.1455,-1.5714,2.0766]
Q84 = [2.0687,-1.2585,1.8296,-2.1443,-1.5714,2.0804]
Q85 = [2.0722,-1.2555,1.8254,-2.1431,-1.5714,2.0843]
Q86 = [2.0758,-1.2524,1.8211,-2.1419,-1.5714,2.0881]
Q87 = [2.0793,-1.2494,1.8168,-2.1406,-1.5714,2.092]
Q88 = [2.0828,-1.2463,1.8124,-2.1394,-1.5714,2.0958]
Q89 = [2.0862,-1.2432,1.8081,-2.1381,-1.5714,2.0996]
Q90 = [2.0897,-1.2401,1.8037,-2.1368,-1.5714,2.1034]
Q91 = [2.0931,-1.237,1.7993,-2.1355,-1.5714,2.1071]
Q92 = [2.0966,-1.2339,1.7949,-2.1342,-1.5715,2.1109]
Q93 = [2.1,-1.2307,1.7904,-2.1328,-1.5715,2.1147]
Q94 = [2.1034,-1.2276,1.7859,-2.1315,-1.5715,2.1184]
Q95 = [2.1067,-1.2244,1.7814,-2.1301,-1.5715,2.1221]
Q96 = [2.1101,-1.2213,1.7769,-2.1288,-1.5715,2.1259]
Q97 = [2.1134,-1.2181,1.7723,-2.1274,-1.5715,2.1296]
Q98 = [2.1168,-1.2149,1.7677,-2.126,-1.5715,2.1333]
Q99 = [2.1201,-1.2117,1.7631,-2.1245,-1.5715,2.1369]
Q100 = [2.1234,-1.2085,1.7584,-2.1231,-1.5715,2.1406]
Q101 = [2.1266,-1.2053,1.7538,-2.1216,-1.5715,2.1442]
Q102 = [2.1299,-1.2021,1.7491,-2.1202,-1.5715,2.1479]
Q103 = [2.1331,-1.1989,1.7444,-2.1187,-1.5715,2.1515]
Q104 = [2.1363,-1.1956,1.7396,-2.1172,-1.5715,2.1551]
Q105 = [2.1396,-1.1924,1.7348,-2.1157,-1.5716,2.1587]
Q106 = [2.1427,-1.1891,1.73,-2.1141,-1.5716,2.1623]
Q107 = [2.1459,-1.1858,1.7252,-2.1126,-1.5716,2.1659]
Q108 = [2.1491,-1.1825,1.7204,-2.111,-1.5716,2.1694]
Q109 = [2.1522,-1.1792,1.7155,-2.1094,-1.5716,2.173]
Q110 = [2.1553,-1.1759,1.7106,-2.1078,-1.5716,2.1765]
Q111 = [2.1585,-1.1726,1.7056,-2.1062,-1.5716,2.18]
Q112 = [2.1615,-1.1693,1.7007,-2.1046,-1.5716,2.1835]
Q113 = [2.1646,-1.1659,1.6957,-2.1029,-1.5716,2.187]
Q114 = [2.1677,-1.1626,1.6907,-2.1013,-1.5716,2.1905]
Q115 = [2.1707,-1.1592,1.6856,-2.0996,-1.5716,2.1939]
Q116 = [2.1738,-1.1558,1.6806,-2.0979,-1.5716,2.1973]
Q117 = [2.1768,-1.1524,1.6755,-2.0962,-1.5716,2.2008]
Q118 = [2.1798,-1.149,1.6703,-2.0944,-1.5716,2.2042]
Q119 = [2.1828,-1.1456,1.6652,-2.0927,-1.5717,2.2076]
Q120 = [2.1857,-1.1422,1.66,-2.0909,-1.5717,2.2109]
Q121 = [2.1887,-1.1388,1.6548,-2.0891,-1.5717,2.2143]
Q122 = [2.1916,-1.1354,1.6495,-2.0873,-1.5717,2.2177]
Q123 = [2.1945,-1.1319,1.6443,-2.0855,-1.5717,2.221]
Q124 = [2.1975,-1.1284,1.639,-2.0837,-1.5717,2.2243]
Q125 = [2.2003,-1.125,1.6337,-2.0818,-1.5717,2.2276]
Q126 = [2.2032,-1.1215,1.6283,-2.0799,-1.5717,2.2309]
Q127 = [2.2061,-1.118,1.6229,-2.078,-1.5717,2.2341]
Q128 = [2.2089,-1.1145,1.6175,-2.0761,-1.5717,2.2374]
Q129 = [2.2118,-1.111,1.6121,-2.0742,-1.5717,2.2406]
Q130 = [2.2146,-1.1074,1.6066,-2.0723,-1.5717,2.2438]
Q131 = [2.2174,-1.1039,1.6011,-2.0703,-1.5717,2.247]
Q132 = [2.2202,-1.1003,1.5956,-2.0683,-1.5717,2.2502]
Q133 = [2.2229,-1.0968,1.59,-2.0663,-1.5718,2.2534]
Q134 = [2.2257,-1.0932,1.5844,-2.0643,-1.5718,2.2565]
Q135 = [2.2284,-1.0896,1.5788,-2.0623,-1.5718,2.2597]
Q136 = [2.2312,-1.086,1.5732,-2.0602,-1.5718,2.2628]
Q137 = [2.2339,-1.0824,1.5675,-2.0582,-1.5718,2.2659]
Q138 = [2.2366,-1.0788,1.5618,-2.0561,-1.5718,2.269]
Q139 = [2.2393,-1.0752,1.556,-2.054,-1.5718,2.272]
Q140 = [2.2419,-1.0715,1.5503,-2.0518,-1.5718,2.2751]
Q141 = [2.2446,-1.0678,1.5445,-2.0497,-1.5718,2.2781]
Q142 = [2.2472,-1.0642,1.5386,-2.0475,-1.5718,2.2811]
Q143 = [2.2499,-1.0605,1.5328,-2.0453,-1.5718,2.2841]
Q144 = [2.2525,-1.0568,1.5269,-2.0431,-1.5718,2.2871]
Q145 = [2.2551,-1.0531,1.5209,-2.0409,-1.5718,2.2901]
Q146 = [2.2577,-1.0494,1.515,-2.0387,-1.5718,2.293]
Q147 = [2.2602,-1.0456,1.509,-2.0364,-1.5718,2.2959]
Q148 = [2.2628,-1.0419,1.5029,-2.0341,-1.5718,2.2989]
Q149 = [2.2654,-1.0381,1.4969,-2.0318,-1.5718,2.3017]
Q150 = [2.2679,-1.0344,1.4908,-2.0295,-1.5719,2.3046]
Q151 = [2.2704,-1.0306,1.4847,-2.0271,-1.5719,2.3075]
Q152 = [2.2729,-1.0268,1.4785,-2.0248,-1.5719,2.3103]
Q153 = [2.2754,-1.023,1.4723,-2.0224,-1.5719,2.3131]
Q154 = [2.2779,-1.0191,1.4661,-2.02,-1.5719,2.3159]
Q155 = [2.2804,-1.0153,1.4598,-2.0176,-1.5719,2.3187]
Q156 = [2.2828,-1.0114,1.4535,-2.0151,-1.5719,2.3215]
Q157 = [2.2853,-1.0076,1.4472,-2.0126,-1.5719,2.3242]
Q158 = [2.2877,-1.0037,1.4408,-2.0101,-1.5719,2.3269]
Q159 = [2.2901,-0.99978,1.4344,-2.0076,-1.5719,2.3296]
Q160 = [2.2925,-0.99586,1.4279,-2.0051,-1.5719,2.3323]
Q161 = [2.2949,-0.99194,1.4215,-2.0025,-1.5719,2.335]
Q162 = [2.2973,-0.98799,1.4149,-2,-1.5719,2.3376]
Q163 = [2.2997,-0.98404,1.4084,-1.9974,-1.5719,2.3402]
Q164 = [2.302,-0.98006,1.4018,-1.9947,-1.5719,2.3428]
Q165 = [2.3043,-0.97608,1.3952,-1.9921,-1.5719,2.3454]
Q166 = [2.3067,-0.97207,1.3885,-1.9894,-1.5719,2.348]
Q167 = [2.309,-0.96805,1.3818,-1.9867,-1.5719,2.3506]
Q168 = [2.3113,-0.96402,1.375,-1.984,-1.5719,2.3531]
Q169 = [2.3136,-0.95996,1.3682,-1.9813,-1.572,2.3556]
Q170 = [2.3159,-0.95589,1.3614,-1.9785,-1.572,2.3581]
Q171 = [2.3182,-0.95181,1.3545,-1.9757,-1.572,2.3606]
Q172 = [2.3204,-0.94771,1.3476,-1.9729,-1.572,2.363]
Q173 = [2.3227,-0.94359,1.3407,-1.9701,-1.572,2.3654]
Q174 = [2.3249,-0.93945,1.3337,-1.9672,-1.572,2.3678]
Q175 = [2.3271,-0.93529,1.3266,-1.9643,-1.572,2.3702]
Q176 = [2.3293,-0.93112,1.3196,-1.9614,-1.572,2.3726]
Q177 = [2.3316,-0.92693,1.3124,-1.9585,-1.572,2.375]
Q178 = [2.3337,-0.92272,1.3053,-1.9555,-1.572,2.3773]
Q179 = [2.3359,-0.91849,1.298,-1.9525,-1.572,2.3796]
Q180 = [2.3381,-0.91425,1.2908,-1.9495,-1.572,2.3819]
Q181 = [2.3403,-0.90998,1.2835,-1.9465,-1.572,2.3842]
Q182 = [2.3424,-0.90569,1.2761,-1.9434,-1.572,2.3864]
Q183 = [2.3445,-0.90139,1.2687,-1.9403,-1.572,2.3886]
Q184 = [2.3467,-0.89706,1.2613,-1.9372,-1.572,2.3908]
Q185 = [2.3488,-0.89271,1.2538,-1.934,-1.572,2.393]
Q186 = [2.3509,-0.88834,1.2462,-1.9308,-1.572,2.3952]
Q187 = [2.353,-0.88395,1.2386,-1.9276,-1.572,2.3973]
Q188 = [2.3551,-0.87954,1.231,-1.9244,-1.572,2.3994]
Q189 = [2.3572,-0.8751,1.2233,-1.9211,-1.572,2.4015]
Q190 = [2.3592,-0.87065,1.2155,-1.9178,-1.5721,2.4036]
Q191 = [2.3613,-0.86617,1.2077,-1.9145,-1.5721,2.4057]
Q192 = [2.3633,-0.86166,1.1998,-1.9111,-1.5721,2.4077]
Q193 = [2.3654,-0.85714,1.1919,-1.9077,-1.5721,2.4097]
Q194 = [2.3674,-0.85258,1.1839,-1.9043,-1.5721,2.4117]
Q195 = [2.3694,-0.84801,1.1759,-1.9008,-1.5721,2.4137]
Q196 = [2.3714,-0.84341,1.1678,-1.8973,-1.5721,2.4156]
Q197 = [2.3734,-0.83878,1.1597,-1.8938,-1.5721,2.4176]
Q198 = [2.3754,-0.83412,1.1515,-1.8903,-1.5721,2.4195]
Q199 = [2.3774,-0.82944,1.1432,-1.8867,-1.5721,2.4213]
Q200 = [2.3794,-0.82473,1.1348,-1.883,-1.5721,2.4232]
Q201 = [2.389,-0.83901,1.1601,-1.894,-1.5721,2.4329]
Q202 = [2.3988,-0.85286,1.1844,-1.9045,-1.5721,2.4427]
Q203 = [2.4088,-0.86631,1.2079,-1.9145,-1.5722,2.4527]
Q204 = [2.419,-0.87939,1.2307,-1.9242,-1.5722,2.4628]
Q205 = [2.4293,-0.89209,1.2527,-1.9334,-1.5722,2.4731]
Q206 = [2.4398,-0.90443,1.2739,-1.9423,-1.5722,2.4836]
Q207 = [2.4505,-0.91642,1.2945,-1.9509,-1.5722,2.4943]
Q208 = [2.4613,-0.92808,1.3143,-1.9591,-1.5723,2.5052]
Q209 = [2.4723,-0.93941,1.3336,-1.9669,-1.5723,2.5162]
Q210 = [2.4835,-0.95041,1.3521,-1.9745,-1.5723,2.5274]


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
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(5.2287)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(5.4573)),
            JointTrajectoryPoint(positions=Q4, velocities=[0]*6, time_from_start=rospy.Duration(5.686)),
            JointTrajectoryPoint(positions=Q5, velocities=[0]*6, time_from_start=rospy.Duration(5.9146)),
            JointTrajectoryPoint(positions=Q6, velocities=[0]*6, time_from_start=rospy.Duration(6.1433)),
            JointTrajectoryPoint(positions=Q7, velocities=[0]*6, time_from_start=rospy.Duration(6.3719)),
            JointTrajectoryPoint(positions=Q8, velocities=[0]*6, time_from_start=rospy.Duration(6.6006)),
            JointTrajectoryPoint(positions=Q9, velocities=[0]*6, time_from_start=rospy.Duration(6.8292)),
            JointTrajectoryPoint(positions=Q10, velocities=[0]*6, time_from_start=rospy.Duration(7.0579)),
            JointTrajectoryPoint(positions=Q11, velocities=[0]*6, time_from_start=rospy.Duration(7.2865)),
            JointTrajectoryPoint(positions=Q12, velocities=[0]*6, time_from_start=rospy.Duration(7.5152)),
            JointTrajectoryPoint(positions=Q13, velocities=[0]*6, time_from_start=rospy.Duration(7.7438)),
            JointTrajectoryPoint(positions=Q14, velocities=[0]*6, time_from_start=rospy.Duration(7.9725)),
            JointTrajectoryPoint(positions=Q15, velocities=[0]*6, time_from_start=rospy.Duration(8.2011)),
            JointTrajectoryPoint(positions=Q16, velocities=[0]*6, time_from_start=rospy.Duration(8.4298)),
            JointTrajectoryPoint(positions=Q17, velocities=[0]*6, time_from_start=rospy.Duration(8.6584)),
            JointTrajectoryPoint(positions=Q18, velocities=[0]*6, time_from_start=rospy.Duration(8.8871)),
            JointTrajectoryPoint(positions=Q19, velocities=[0]*6, time_from_start=rospy.Duration(9.1157)),
            JointTrajectoryPoint(positions=Q20, velocities=[0]*6, time_from_start=rospy.Duration(9.3444)),
            JointTrajectoryPoint(positions=Q21, velocities=[0]*6, time_from_start=rospy.Duration(9.5731)),
            JointTrajectoryPoint(positions=Q22, velocities=[0]*6, time_from_start=rospy.Duration(9.8017)),
            JointTrajectoryPoint(positions=Q23, velocities=[0]*6, time_from_start=rospy.Duration(10.0304)),
            JointTrajectoryPoint(positions=Q24, velocities=[0]*6, time_from_start=rospy.Duration(10.259)),
            JointTrajectoryPoint(positions=Q25, velocities=[0]*6, time_from_start=rospy.Duration(10.4877)),
            JointTrajectoryPoint(positions=Q26, velocities=[0]*6, time_from_start=rospy.Duration(10.7163)),
            JointTrajectoryPoint(positions=Q27, velocities=[0]*6, time_from_start=rospy.Duration(10.945)),
            JointTrajectoryPoint(positions=Q28, velocities=[0]*6, time_from_start=rospy.Duration(11.1736)),
            JointTrajectoryPoint(positions=Q29, velocities=[0]*6, time_from_start=rospy.Duration(11.4023)),
            JointTrajectoryPoint(positions=Q30, velocities=[0]*6, time_from_start=rospy.Duration(11.6309)),
            JointTrajectoryPoint(positions=Q31, velocities=[0]*6, time_from_start=rospy.Duration(11.8596)),
            JointTrajectoryPoint(positions=Q32, velocities=[0]*6, time_from_start=rospy.Duration(12.0882)),
            JointTrajectoryPoint(positions=Q33, velocities=[0]*6, time_from_start=rospy.Duration(12.3169)),
            JointTrajectoryPoint(positions=Q34, velocities=[0]*6, time_from_start=rospy.Duration(12.5455)),
            JointTrajectoryPoint(positions=Q35, velocities=[0]*6, time_from_start=rospy.Duration(12.7742)),
            JointTrajectoryPoint(positions=Q36, velocities=[0]*6, time_from_start=rospy.Duration(13.0028)),
            JointTrajectoryPoint(positions=Q37, velocities=[0]*6, time_from_start=rospy.Duration(13.2315)),
            JointTrajectoryPoint(positions=Q38, velocities=[0]*6, time_from_start=rospy.Duration(13.4601)),
            JointTrajectoryPoint(positions=Q39, velocities=[0]*6, time_from_start=rospy.Duration(13.6888)),
            JointTrajectoryPoint(positions=Q40, velocities=[0]*6, time_from_start=rospy.Duration(13.9174)),
            JointTrajectoryPoint(positions=Q41, velocities=[0]*6, time_from_start=rospy.Duration(14.1461)),
            JointTrajectoryPoint(positions=Q42, velocities=[0]*6, time_from_start=rospy.Duration(14.3748)),
            JointTrajectoryPoint(positions=Q43, velocities=[0]*6, time_from_start=rospy.Duration(14.6034)),
            JointTrajectoryPoint(positions=Q44, velocities=[0]*6, time_from_start=rospy.Duration(14.8321)),
            JointTrajectoryPoint(positions=Q45, velocities=[0]*6, time_from_start=rospy.Duration(15.0607)),
            JointTrajectoryPoint(positions=Q46, velocities=[0]*6, time_from_start=rospy.Duration(15.2894)),
            JointTrajectoryPoint(positions=Q47, velocities=[0]*6, time_from_start=rospy.Duration(15.518)),
            JointTrajectoryPoint(positions=Q48, velocities=[0]*6, time_from_start=rospy.Duration(15.7467)),
            JointTrajectoryPoint(positions=Q49, velocities=[0]*6, time_from_start=rospy.Duration(15.9753)),
            JointTrajectoryPoint(positions=Q50, velocities=[0]*6, time_from_start=rospy.Duration(16.204)),
            JointTrajectoryPoint(positions=Q51, velocities=[0]*6, time_from_start=rospy.Duration(16.4326)),
            JointTrajectoryPoint(positions=Q52, velocities=[0]*6, time_from_start=rospy.Duration(16.6613)),
            JointTrajectoryPoint(positions=Q53, velocities=[0]*6, time_from_start=rospy.Duration(16.8899)),
            JointTrajectoryPoint(positions=Q54, velocities=[0]*6, time_from_start=rospy.Duration(17.1186)),
            JointTrajectoryPoint(positions=Q55, velocities=[0]*6, time_from_start=rospy.Duration(17.3472)),
            JointTrajectoryPoint(positions=Q56, velocities=[0]*6, time_from_start=rospy.Duration(17.5759)),
            JointTrajectoryPoint(positions=Q57, velocities=[0]*6, time_from_start=rospy.Duration(17.8045)),
            JointTrajectoryPoint(positions=Q58, velocities=[0]*6, time_from_start=rospy.Duration(18.0332)),
            JointTrajectoryPoint(positions=Q59, velocities=[0]*6, time_from_start=rospy.Duration(18.2618)),
            JointTrajectoryPoint(positions=Q60, velocities=[0]*6, time_from_start=rospy.Duration(18.4905)),
            JointTrajectoryPoint(positions=Q61, velocities=[0]*6, time_from_start=rospy.Duration(18.7192)),
            JointTrajectoryPoint(positions=Q62, velocities=[0]*6, time_from_start=rospy.Duration(18.9478)),
            JointTrajectoryPoint(positions=Q63, velocities=[0]*6, time_from_start=rospy.Duration(19.1765)),
            JointTrajectoryPoint(positions=Q64, velocities=[0]*6, time_from_start=rospy.Duration(19.4051)),
            JointTrajectoryPoint(positions=Q65, velocities=[0]*6, time_from_start=rospy.Duration(19.6338)),
            JointTrajectoryPoint(positions=Q66, velocities=[0]*6, time_from_start=rospy.Duration(19.8624)),
            JointTrajectoryPoint(positions=Q67, velocities=[0]*6, time_from_start=rospy.Duration(20.0911)),
            JointTrajectoryPoint(positions=Q68, velocities=[0]*6, time_from_start=rospy.Duration(20.3197)),
            JointTrajectoryPoint(positions=Q69, velocities=[0]*6, time_from_start=rospy.Duration(20.5484)),
            JointTrajectoryPoint(positions=Q70, velocities=[0]*6, time_from_start=rospy.Duration(20.777)),
            JointTrajectoryPoint(positions=Q71, velocities=[0]*6, time_from_start=rospy.Duration(21.0057)),
            JointTrajectoryPoint(positions=Q72, velocities=[0]*6, time_from_start=rospy.Duration(21.2343)),
            JointTrajectoryPoint(positions=Q73, velocities=[0]*6, time_from_start=rospy.Duration(21.463)),
            JointTrajectoryPoint(positions=Q74, velocities=[0]*6, time_from_start=rospy.Duration(21.6916)),
            JointTrajectoryPoint(positions=Q75, velocities=[0]*6, time_from_start=rospy.Duration(21.9203)),
            JointTrajectoryPoint(positions=Q76, velocities=[0]*6, time_from_start=rospy.Duration(22.1489)),
            JointTrajectoryPoint(positions=Q77, velocities=[0]*6, time_from_start=rospy.Duration(22.3776)),
            JointTrajectoryPoint(positions=Q78, velocities=[0]*6, time_from_start=rospy.Duration(22.6062)),
            JointTrajectoryPoint(positions=Q79, velocities=[0]*6, time_from_start=rospy.Duration(22.8349)),
            JointTrajectoryPoint(positions=Q80, velocities=[0]*6, time_from_start=rospy.Duration(23.0636)),
            JointTrajectoryPoint(positions=Q81, velocities=[0]*6, time_from_start=rospy.Duration(23.2922)),
            JointTrajectoryPoint(positions=Q82, velocities=[0]*6, time_from_start=rospy.Duration(23.5209)),
            JointTrajectoryPoint(positions=Q83, velocities=[0]*6, time_from_start=rospy.Duration(23.7495)),
            JointTrajectoryPoint(positions=Q84, velocities=[0]*6, time_from_start=rospy.Duration(23.9782)),
            JointTrajectoryPoint(positions=Q85, velocities=[0]*6, time_from_start=rospy.Duration(24.2068)),
            JointTrajectoryPoint(positions=Q86, velocities=[0]*6, time_from_start=rospy.Duration(24.4355)),
            JointTrajectoryPoint(positions=Q87, velocities=[0]*6, time_from_start=rospy.Duration(24.6641)),
            JointTrajectoryPoint(positions=Q88, velocities=[0]*6, time_from_start=rospy.Duration(24.8928)),
            JointTrajectoryPoint(positions=Q89, velocities=[0]*6, time_from_start=rospy.Duration(25.1214)),
            JointTrajectoryPoint(positions=Q90, velocities=[0]*6, time_from_start=rospy.Duration(25.3501)),
            JointTrajectoryPoint(positions=Q91, velocities=[0]*6, time_from_start=rospy.Duration(25.5787)),
            JointTrajectoryPoint(positions=Q92, velocities=[0]*6, time_from_start=rospy.Duration(25.8074)),
            JointTrajectoryPoint(positions=Q93, velocities=[0]*6, time_from_start=rospy.Duration(26.036)),
            JointTrajectoryPoint(positions=Q94, velocities=[0]*6, time_from_start=rospy.Duration(26.2647)),
            JointTrajectoryPoint(positions=Q95, velocities=[0]*6, time_from_start=rospy.Duration(26.4933)),
            JointTrajectoryPoint(positions=Q96, velocities=[0]*6, time_from_start=rospy.Duration(26.722)),
            JointTrajectoryPoint(positions=Q97, velocities=[0]*6, time_from_start=rospy.Duration(26.9506)),
            JointTrajectoryPoint(positions=Q98, velocities=[0]*6, time_from_start=rospy.Duration(27.1793)),
            JointTrajectoryPoint(positions=Q99, velocities=[0]*6, time_from_start=rospy.Duration(27.408)),
            JointTrajectoryPoint(positions=Q100, velocities=[0]*6, time_from_start=rospy.Duration(27.6366)),
            JointTrajectoryPoint(positions=Q101, velocities=[0]*6, time_from_start=rospy.Duration(27.8653)),
            JointTrajectoryPoint(positions=Q102, velocities=[0]*6, time_from_start=rospy.Duration(28.0939)),
            JointTrajectoryPoint(positions=Q103, velocities=[0]*6, time_from_start=rospy.Duration(28.3226)),
            JointTrajectoryPoint(positions=Q104, velocities=[0]*6, time_from_start=rospy.Duration(28.5512)),
            JointTrajectoryPoint(positions=Q105, velocities=[0]*6, time_from_start=rospy.Duration(28.7799)),
            JointTrajectoryPoint(positions=Q106, velocities=[0]*6, time_from_start=rospy.Duration(29.0085)),
            JointTrajectoryPoint(positions=Q107, velocities=[0]*6, time_from_start=rospy.Duration(29.2372)),
            JointTrajectoryPoint(positions=Q108, velocities=[0]*6, time_from_start=rospy.Duration(29.4658)),
            JointTrajectoryPoint(positions=Q109, velocities=[0]*6, time_from_start=rospy.Duration(29.6945)),
            JointTrajectoryPoint(positions=Q110, velocities=[0]*6, time_from_start=rospy.Duration(29.9231)),
            JointTrajectoryPoint(positions=Q111, velocities=[0]*6, time_from_start=rospy.Duration(30.1518)),
            JointTrajectoryPoint(positions=Q112, velocities=[0]*6, time_from_start=rospy.Duration(30.3804)),
            JointTrajectoryPoint(positions=Q113, velocities=[0]*6, time_from_start=rospy.Duration(30.6091)),
            JointTrajectoryPoint(positions=Q114, velocities=[0]*6, time_from_start=rospy.Duration(30.8377)),
            JointTrajectoryPoint(positions=Q115, velocities=[0]*6, time_from_start=rospy.Duration(31.0664)),
            JointTrajectoryPoint(positions=Q116, velocities=[0]*6, time_from_start=rospy.Duration(31.295)),
            JointTrajectoryPoint(positions=Q117, velocities=[0]*6, time_from_start=rospy.Duration(31.5237)),
            JointTrajectoryPoint(positions=Q118, velocities=[0]*6, time_from_start=rospy.Duration(31.7523)),
            JointTrajectoryPoint(positions=Q119, velocities=[0]*6, time_from_start=rospy.Duration(31.981)),
            JointTrajectoryPoint(positions=Q120, velocities=[0]*6, time_from_start=rospy.Duration(32.2097)),
            JointTrajectoryPoint(positions=Q121, velocities=[0]*6, time_from_start=rospy.Duration(32.4383)),
            JointTrajectoryPoint(positions=Q122, velocities=[0]*6, time_from_start=rospy.Duration(32.667)),
            JointTrajectoryPoint(positions=Q123, velocities=[0]*6, time_from_start=rospy.Duration(32.8956)),
            JointTrajectoryPoint(positions=Q124, velocities=[0]*6, time_from_start=rospy.Duration(33.1243)),
            JointTrajectoryPoint(positions=Q125, velocities=[0]*6, time_from_start=rospy.Duration(33.3529)),
            JointTrajectoryPoint(positions=Q126, velocities=[0]*6, time_from_start=rospy.Duration(33.5816)),
            JointTrajectoryPoint(positions=Q127, velocities=[0]*6, time_from_start=rospy.Duration(33.8102)),
            JointTrajectoryPoint(positions=Q128, velocities=[0]*6, time_from_start=rospy.Duration(34.0389)),
            JointTrajectoryPoint(positions=Q129, velocities=[0]*6, time_from_start=rospy.Duration(34.2675)),
            JointTrajectoryPoint(positions=Q130, velocities=[0]*6, time_from_start=rospy.Duration(34.4962)),
            JointTrajectoryPoint(positions=Q131, velocities=[0]*6, time_from_start=rospy.Duration(34.7248)),
            JointTrajectoryPoint(positions=Q132, velocities=[0]*6, time_from_start=rospy.Duration(34.9535)),
            JointTrajectoryPoint(positions=Q133, velocities=[0]*6, time_from_start=rospy.Duration(35.1821)),
            JointTrajectoryPoint(positions=Q134, velocities=[0]*6, time_from_start=rospy.Duration(35.4108)),
            JointTrajectoryPoint(positions=Q135, velocities=[0]*6, time_from_start=rospy.Duration(35.6394)),
            JointTrajectoryPoint(positions=Q136, velocities=[0]*6, time_from_start=rospy.Duration(35.8681)),
            JointTrajectoryPoint(positions=Q137, velocities=[0]*6, time_from_start=rospy.Duration(36.0967)),
            JointTrajectoryPoint(positions=Q138, velocities=[0]*6, time_from_start=rospy.Duration(36.3254)),
            JointTrajectoryPoint(positions=Q139, velocities=[0]*6, time_from_start=rospy.Duration(36.5541)),
            JointTrajectoryPoint(positions=Q140, velocities=[0]*6, time_from_start=rospy.Duration(36.7827)),
            JointTrajectoryPoint(positions=Q141, velocities=[0]*6, time_from_start=rospy.Duration(37.0114)),
            JointTrajectoryPoint(positions=Q142, velocities=[0]*6, time_from_start=rospy.Duration(37.24)),
            JointTrajectoryPoint(positions=Q143, velocities=[0]*6, time_from_start=rospy.Duration(37.4687)),
            JointTrajectoryPoint(positions=Q144, velocities=[0]*6, time_from_start=rospy.Duration(37.6973)),
            JointTrajectoryPoint(positions=Q145, velocities=[0]*6, time_from_start=rospy.Duration(37.926)),
            JointTrajectoryPoint(positions=Q146, velocities=[0]*6, time_from_start=rospy.Duration(38.1546)),
            JointTrajectoryPoint(positions=Q147, velocities=[0]*6, time_from_start=rospy.Duration(38.3833)),
            JointTrajectoryPoint(positions=Q148, velocities=[0]*6, time_from_start=rospy.Duration(38.6119)),
            JointTrajectoryPoint(positions=Q149, velocities=[0]*6, time_from_start=rospy.Duration(38.8406)),
            JointTrajectoryPoint(positions=Q150, velocities=[0]*6, time_from_start=rospy.Duration(39.0692)),
            JointTrajectoryPoint(positions=Q151, velocities=[0]*6, time_from_start=rospy.Duration(39.2979)),
            JointTrajectoryPoint(positions=Q152, velocities=[0]*6, time_from_start=rospy.Duration(39.5265)),
            JointTrajectoryPoint(positions=Q153, velocities=[0]*6, time_from_start=rospy.Duration(39.7552)),
            JointTrajectoryPoint(positions=Q154, velocities=[0]*6, time_from_start=rospy.Duration(39.9838)),
            JointTrajectoryPoint(positions=Q155, velocities=[0]*6, time_from_start=rospy.Duration(40.2125)),
            JointTrajectoryPoint(positions=Q156, velocities=[0]*6, time_from_start=rospy.Duration(40.4411)),
            JointTrajectoryPoint(positions=Q157, velocities=[0]*6, time_from_start=rospy.Duration(40.6698)),
            JointTrajectoryPoint(positions=Q158, velocities=[0]*6, time_from_start=rospy.Duration(40.8985)),
            JointTrajectoryPoint(positions=Q159, velocities=[0]*6, time_from_start=rospy.Duration(41.1271)),
            JointTrajectoryPoint(positions=Q160, velocities=[0]*6, time_from_start=rospy.Duration(41.3558)),
            JointTrajectoryPoint(positions=Q161, velocities=[0]*6, time_from_start=rospy.Duration(41.5844)),
            JointTrajectoryPoint(positions=Q162, velocities=[0]*6, time_from_start=rospy.Duration(41.8131)),
            JointTrajectoryPoint(positions=Q163, velocities=[0]*6, time_from_start=rospy.Duration(42.0417)),
            JointTrajectoryPoint(positions=Q164, velocities=[0]*6, time_from_start=rospy.Duration(42.2704)),
            JointTrajectoryPoint(positions=Q165, velocities=[0]*6, time_from_start=rospy.Duration(42.499)),
            JointTrajectoryPoint(positions=Q166, velocities=[0]*6, time_from_start=rospy.Duration(42.7277)),
            JointTrajectoryPoint(positions=Q167, velocities=[0]*6, time_from_start=rospy.Duration(42.9563)),
            JointTrajectoryPoint(positions=Q168, velocities=[0]*6, time_from_start=rospy.Duration(43.185)),
            JointTrajectoryPoint(positions=Q169, velocities=[0]*6, time_from_start=rospy.Duration(43.4136)),
            JointTrajectoryPoint(positions=Q170, velocities=[0]*6, time_from_start=rospy.Duration(43.6423)),
            JointTrajectoryPoint(positions=Q171, velocities=[0]*6, time_from_start=rospy.Duration(43.8709)),
            JointTrajectoryPoint(positions=Q172, velocities=[0]*6, time_from_start=rospy.Duration(44.0996)),
            JointTrajectoryPoint(positions=Q173, velocities=[0]*6, time_from_start=rospy.Duration(44.3282)),
            JointTrajectoryPoint(positions=Q174, velocities=[0]*6, time_from_start=rospy.Duration(44.5569)),
            JointTrajectoryPoint(positions=Q175, velocities=[0]*6, time_from_start=rospy.Duration(44.7855)),
            JointTrajectoryPoint(positions=Q176, velocities=[0]*6, time_from_start=rospy.Duration(45.0142)),
            JointTrajectoryPoint(positions=Q177, velocities=[0]*6, time_from_start=rospy.Duration(45.2429)),
            JointTrajectoryPoint(positions=Q178, velocities=[0]*6, time_from_start=rospy.Duration(45.4715)),
            JointTrajectoryPoint(positions=Q179, velocities=[0]*6, time_from_start=rospy.Duration(45.7002)),
            JointTrajectoryPoint(positions=Q180, velocities=[0]*6, time_from_start=rospy.Duration(45.9288)),
            JointTrajectoryPoint(positions=Q181, velocities=[0]*6, time_from_start=rospy.Duration(46.1575)),
            JointTrajectoryPoint(positions=Q182, velocities=[0]*6, time_from_start=rospy.Duration(46.3861)),
            JointTrajectoryPoint(positions=Q183, velocities=[0]*6, time_from_start=rospy.Duration(46.6148)),
            JointTrajectoryPoint(positions=Q184, velocities=[0]*6, time_from_start=rospy.Duration(46.8434)),
            JointTrajectoryPoint(positions=Q185, velocities=[0]*6, time_from_start=rospy.Duration(47.0721)),
            JointTrajectoryPoint(positions=Q186, velocities=[0]*6, time_from_start=rospy.Duration(47.3007)),
            JointTrajectoryPoint(positions=Q187, velocities=[0]*6, time_from_start=rospy.Duration(47.5294)),
            JointTrajectoryPoint(positions=Q188, velocities=[0]*6, time_from_start=rospy.Duration(47.758)),
            JointTrajectoryPoint(positions=Q189, velocities=[0]*6, time_from_start=rospy.Duration(47.9867)),
            JointTrajectoryPoint(positions=Q190, velocities=[0]*6, time_from_start=rospy.Duration(48.2153)),
            JointTrajectoryPoint(positions=Q191, velocities=[0]*6, time_from_start=rospy.Duration(48.444)),
            JointTrajectoryPoint(positions=Q192, velocities=[0]*6, time_from_start=rospy.Duration(48.6726)),
            JointTrajectoryPoint(positions=Q193, velocities=[0]*6, time_from_start=rospy.Duration(48.9013)),
            JointTrajectoryPoint(positions=Q194, velocities=[0]*6, time_from_start=rospy.Duration(49.1299)),
            JointTrajectoryPoint(positions=Q195, velocities=[0]*6, time_from_start=rospy.Duration(49.3586)),
            JointTrajectoryPoint(positions=Q196, velocities=[0]*6, time_from_start=rospy.Duration(49.5872)),
            JointTrajectoryPoint(positions=Q197, velocities=[0]*6, time_from_start=rospy.Duration(49.8159)),
            JointTrajectoryPoint(positions=Q198, velocities=[0]*6, time_from_start=rospy.Duration(50.0446)),
            JointTrajectoryPoint(positions=Q199, velocities=[0]*6, time_from_start=rospy.Duration(50.2732)),
            JointTrajectoryPoint(positions=Q200, velocities=[0]*6, time_from_start=rospy.Duration(50.5019)),
            JointTrajectoryPoint(positions=Q201, velocities=[0]*6, time_from_start=rospy.Duration(50.6019)),
            JointTrajectoryPoint(positions=Q202, velocities=[0]*6, time_from_start=rospy.Duration(50.7019)),
            JointTrajectoryPoint(positions=Q203, velocities=[0]*6, time_from_start=rospy.Duration(50.8019)),
            JointTrajectoryPoint(positions=Q204, velocities=[0]*6, time_from_start=rospy.Duration(50.9019)),
            JointTrajectoryPoint(positions=Q205, velocities=[0]*6, time_from_start=rospy.Duration(51.0019)),
            JointTrajectoryPoint(positions=Q206, velocities=[0]*6, time_from_start=rospy.Duration(51.1019)),
            JointTrajectoryPoint(positions=Q207, velocities=[0]*6, time_from_start=rospy.Duration(51.2019)),
            JointTrajectoryPoint(positions=Q208, velocities=[0]*6, time_from_start=rospy.Duration(51.3019)),
            JointTrajectoryPoint(positions=Q209, velocities=[0]*6, time_from_start=rospy.Duration(51.4019)),
            JointTrajectoryPoint(positions=Q210, velocities=[0]*6, time_from_start=rospy.Duration(51.5019))]
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
        child1 = subprocess.Popen('rostopic echo /joint_states >1.txt',shell=True)
        move()
        print "Trajectory finished"
        time.sleep(0.5)
        child2 = subprocess.Popen('cp 1.txt data/polish.txt',shell=True)
        time.sleep(0.5)
        child3 = subprocess.Popen('rm 1.txt',shell=True)
        time.sleep(0.5)
        if True:
            child1.kill()
            child2.kill()
            child3.kill()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
