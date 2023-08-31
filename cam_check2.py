#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils  as tu



def printTr(h_mat1,h_mat2):
    print("pos")
    pos1 = tu.get_pos_mat_from_homogeneous(h_mat1)
    pos2 = tu.get_pos_mat_from_homogeneous(h_mat2)
    print(f"{pos1} \tnorm: {np.linalg.norm(pos1)}")
    print(f"{pos2} \tnorm: {np.linalg.norm(pos2)}")
    print("rot")
    rpy1 = tu.get_rpy_from_matrix(h_mat1)
    rpy2 = tu.get_rpy_from_matrix(h_mat2)
    print(f"{rpy1} \tnorm: {np.linalg.norm(rpy1)}")
    print(f"{rpy2} \tnorm: {np.linalg.norm(rpy2)}")
    print(f"pose error: {tu.compute_pose_error(h_mat1,h_mat2)}")
    print()

tcp1 = Transform(
pos=[0.3059400143833206, -0.09819697255591989,0.4842033981548661],
rot=[0.0028022988806761565, -0.9999772861914895, 0.003987576968235117, 0.004655475486904665]).h_mat
cam1 = Transform(
pos=[0.3561722216834493, -0.13139413352358756,0.5369525726242711],
rot=[-0.005271197099218794, 0.7099103794917093, 0.7042710815255532, -0.00130816585566478]).h_mat

tcp2 = Transform(
pos=[0.30770208113644226, -0.05102083700767337,0.4862429289862656],
rot=[-0.003312732765945259, -0.9999887114194103, -0.0033051498038210673, 0.0008239054204972164]).h_mat
cam2 = Transform(
pos=[0.35800275319029845, -0.08283508914081944,0.5397732690365502],
rot=[0.0017620961345398564, 0.7047616995448592, 0.709435902043312, -0.0029228006852409613]).h_mat

tcp3 = Transform(
pos=[0.30804280939335144, -0.000939410918797783,0.4862150064992427],
rot=[-0.0033851104672825813, -0.9999852843416999, -0.004238080888572788, 0.00010391128828893073]).h_mat
cam3 = Transform(
pos=[0.35832607160425434, -0.032652160946679,0.5398218771489905],
rot=[0.0023223855367564103, 0.7040995925661417, 0.7100931591812005, -0.002464864636196634]).h_mat

tcp4 = Transform(
pos=[0.3067721224715857, 0.04911027727063169,0.48512646163274764],
rot=[-0.003489896687083558, -0.9999791150808197, -0.005146190467670762, 0.001762596654128695]).h_mat
cam4 = Transform(
pos=[0.3572903817192108, 0.017500464888989535,0.5385729469947171],
rot=[0.0012236108682042806, 0.7034531032743555, 0.7107309312410733, -0.0037118253353127014]).h_mat

tcp5 = Transform(
pos=[0.3059788326861551, 0.09823812950504493,0.484478967883054],
rot=[-0.004691772954576352, -0.9999717753718305, -0.004411603766003452, 0.003869557915792342]).h_mat
cam5 = Transform(
pos=[0.3566743612856194, 0.06668304743719103,0.5377897710046723],
rot=[0.000583620758028469, 0.7039673465790215, 0.7102063174463649, -0.006051528449597424]).h_mat

#pnp
pnp1=Transform(pos=[0.02236147, 0.08304681, 0.51968909], rot=[ 0.71334804, -0.00983261, -0.01998483, -0.70045592]).h_mat
pnp2=Transform(pos=[-0.02088335,  0.07811348,  0.52423864], rot=[ 0.71870306, -0.01219437, -0.01246585, -0.69509842]).h_mat
pnp3=Transform(pos=[-0.07114245,  0.07787712,  0.52298441], rot=[ 0.71931948, -0.00758714, -0.0160603,  -0.6944523 ]).h_mat
pnp4=Transform(pos=[-0.12128483,  0.08110133,  0.51921536], rot=[ 0.71995461, -0.00753203, -0.02133075, -0.69365238]).h_mat
pnp5=Transform(pos=[-0.16883497,  0.08513284,  0.51422501], rot=[ 0.71929094, -0.00889965, -0.02822287, -0.69407839]).h_mat

# #calib
# pnp1=Transform(pos=[ 0.06156721,  0.04915918, 11.3554364 ], rot=[ 0.6184781,  -0.35723166, -0.34917838, -0.60658456]).h_mat
# pnp2=Transform(pos=[ 0.01750669,  0.04648098, 11.3423492 ], rot=[ 0.62307028, -0.36099999, -0.3465213,  -0.60115342]).h_mat
# pnp3=Transform(pos=[-0.03292481,  0.04641302, 11.350223  ], rot=[ 0.62373197, -0.36100223, -0.34603709, -0.60074466]).h_mat
# pnp4=Transform(pos=[-0.08339917,  0.0480514,  11.3162818 ], rot=[ 0.62443486, -0.36141787, -0.34544993, -0.60010213]).h_mat
# pnp5=Transform(pos=[-0.13138351,  0.05005515, 11.27502275], rot=[ 0.62421205, -0.36153745, -0.34543739, -0.6002691 ]).h_mat

cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam4t3 = np.dot(np.linalg.inv(cam4),cam3)
cam5t4 = np.dot(np.linalg.inv(cam5),cam4)
cam1t5 = np.dot(np.linalg.inv(cam1),cam5)

tcp2t1 = np.dot(np.linalg.inv(tcp2),tcp1)
tcp3t2 = np.dot(np.linalg.inv(tcp3),tcp2)
tcp4t3 = np.dot(np.linalg.inv(tcp4),tcp3)
tcp5t4 = np.dot(np.linalg.inv(tcp5),tcp4)
tcp1t5 = np.dot(np.linalg.inv(tcp1),tcp5)

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)
pnp3t2=(np.linalg.inv(pnp3)).dot(pnp2)
pnp4t3=(np.linalg.inv(pnp4)).dot(pnp3)
pnp5t4=(np.linalg.inv(pnp5)).dot(pnp4)
pnp1t5=(np.linalg.inv(pnp1)).dot(pnp5)

print("####2t1####")
printTr(cam2t1,pnp2t1)
print("####3t2####")
printTr(cam3t2,pnp3t2)
print("####4t3####")
printTr(cam4t3,pnp4t3)
print("####5t4####")
printTr(cam5t4,pnp5t4)
print("####1t5####")
printTr(cam1t5,pnp1t5)

# print("####2t1####")
# printTr(tcp2t1,pnp2t1)
# print("####3t2####")
# printTr(tcp3t2,pnp3t2)
# print("####4t3####")
# printTr(tcp4t3,pnp4t3)
# print("####5t4####")
# printTr(tcp5t4,pnp5t4)
# print("####1t5####")
# printTr(tcp1t5,pnp1t5)