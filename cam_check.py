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
pos=[0.30532033697889377, -0.07341173605031283,0.484209502106298],
rot=[0.0025548186046679823, -0.9999743407194573, 0.00375045696421095, 0.005543002538661234]).h_mat
cam1 = Transform(
pos=[0.35567256060711905, -0.1062618231460404,0.5373191296714057],
rot=[0.008892651766061346, -0.7149628466421815, -0.6991038509192155, 0.0016911260913460135]).h_mat

tcp2 = Transform(
pos=[0.30687376741886124, -0.03874368344472677,0.48564284082044207],
rot=[-0.003076879423263596, -0.9999850917819313, -0.0033930117426807047, 0.0029726248017083855]).h_mat
cam2 = Transform(
pos=[0.3574131535741803, -0.0702720512231348,0.5393721592497795],
rot=[0.0030802296249658164, -0.709973900977528, -0.7042108126395286, 0.003835243074540093]).h_mat

tcp3 = Transform(
pos=[0.30726110649444033, -0.0013355795817951211,0.48579364298261674],
rot=[-0.003250292823133919, -0.9999869660833479, -0.0034285135937380754, 0.0019361184164397034]).h_mat
cam3 = Transform(
pos=[0.3576914118572194, -0.032841725165895674,0.539638366754389],
rot=[0.0022292868673112042, -0.7099504036735191, -0.7042407970243578, 0.0032186988091689834]).h_mat

tcp4 = Transform(
pos=[0.3063512112075071, 0.03632967714877317,0.48497735828510324],
rot=[-0.0037246432333466867, -0.9999809634285378, -0.003980802256003864, 0.0028901603271543137]).h_mat
cam4 = Transform(
pos=[0.3569188171608849, 0.0049303819866848946,0.5387556873598907],
rot=[0.0025611703306469722, -0.7095604668884743, -0.7046271863305787, 0.0042329813437491735]).h_mat

tcp5 = Transform(
pos=[0.30536298974927534, 0.07365785803552653,0.48452317429911207],
rot=[-0.003987663695494846, -0.9999713291186919, -0.004066521596732688, 0.0049902786447269575]).h_mat
cam5 = Transform(
pos=[0.3561609262733012, 0.042295652197668246,0.5381057169458828],
rot=[0.0038477035076643893, -0.7094952328540763, -0.7046749189418038, 0.005913878682913706]).h_mat



#pnp
pnp1=Transform(pos=[0.03215606, 0.08907264, 0.51871231], rot=[ 0.7070116,  -0.00978385, -0.02126902, -0.70681433]).h_mat
pnp2=Transform(pos=[0.00096874, 0.08553863, 0.52208552], rot=[ 0.71224798, -0.01426128, -0.01484048, -0.70162611]).h_mat
pnp3=Transform(pos=[-0.0360664,   0.08500317,  0.52246682], rot=[ 0.71220911, -0.01233988, -0.01535302, -0.70169095]).h_mat
pnp4=Transform(pos=[-0.07319268,  0.08728011,  0.52008287], rot=[ 0.7125594,  -0.01132666, -0.01973314, -0.70124276]).h_mat
pnp5=Transform(pos=[-0.11014827,  0.0908604,   0.51748807], rot=[ 0.71256452, -0.01178486, -0.02375276, -0.70110536]).h_mat

# #cal
# pnp1=Transform(pos=[0.02487861, 0.10062795, 0.72019599], rot=[ 0.70669047, -0.0113596,  -0.03140712, -0.70673413]).h_mat
# pnp2=Transform(pos=[-0.00667239,  0.09714603,  0.72289947], rot=[ 0.71196122, -0.01555956, -0.02300542, -0.70166935]).h_mat
# pnp3=Transform(pos=[-0.04400624,  0.09660244,  0.72287849], rot=[ 0.71197184, -0.01596042, -0.02093322, -0.70171444]).h_mat
# pnp4=Transform(pos=[-0.08132563,  0.0988119,   0.72001882], rot=[ 0.71230775, -0.01869758, -0.02344513, -0.70122635]).h_mat
# pnp5=Transform(pos=[-0.11841677,  0.10235816,  0.71785693], rot=[ 0.71231492, -0.02172008, -0.02466844, -0.70108998]).h_mat


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