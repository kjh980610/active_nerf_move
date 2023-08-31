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
pos=[0.2077751796795863, -0.0012310390644497738,0.4871389843252457],
rot=[-0.0031609352812599777, -0.999982870929001, -0.004750412827579239, -0.0013038077726252259]).h_mat
cam1 = Transform(
pos=[0.25794040895132603, -0.03291655740047674,0.5408723934265783],
rot=[0.003159276285106586, 0.7037356099899467, 0.7104537223786854, -0.0013109400073979159]).h_mat

tcp2 = Transform(
pos=[0.2555142865021533, -0.0011525481685075523,0.48454702689600737],
rot=[-0.003163966982574464, -0.9999655049101281, -0.005048731303248259, 0.005786934853039579]).h_mat
cam2 = Transform(
pos=[0.30645405140006554, -0.03280619604281068,0.5375657828428906],
rot=[-0.0018524928787084843, 0.7035124031973572, 0.7106524016338538, -0.0063269953097355855]).h_mat

tcp3 = Transform(
pos=[0.30524356583079165, -0.0010705575767168511,0.4841889032946591],
rot=[-0.0031688683948511817, -0.9999600428732841, -0.005296221211841467, 0.006466913547544716]).h_mat
cam3 = Transform(
pos=[0.35627099671553425, -0.032698450900991896,0.53713868341631],
rot=[-0.002329845168612694, 0.7033335406714785, 0.7108235427151838, -0.00681127813615589]).h_mat

tcp4 = Transform(
pos=[0.3551717335429501, -0.0009908302944383895,0.48391801223267894],
rot=[-0.0031619269790503154, -0.9999577309682901, -0.005482951574163065, 0.006669013162628689]).h_mat
cam4 = Transform(
pos=[0.40623239502116654, -0.032600478814008425,0.5368466483726392],
rot=[-0.00247765992195775, 0.7031998680699251, 0.710953946693477, -0.0069492754097539204]).h_mat

tcp5 = Transform(
pos=[0.40485565722674627, -0.0009218003322353436,0.48320739752966735],
rot=[-0.003146703499398193, -0.9999491577973724, -0.005602392306745798, 0.0077713112055660186]).h_mat
cam5 = Transform(
pos=[0.45604025554371835, -0.03252065103722019,0.5360226517940783],
rot=[-0.0032678672550479804, 0.7031093510704131, 0.7110323443297769, -0.007717952957109746]).h_mat

#pnp
pnp1=Transform(pos=[-0.06524508,  0.16206622,  0.5187526 ], rot=[ 0.70607263, -0.02173044, -0.03281554, -0.70704482]).h_mat
pnp2=Transform(pos=[-0.06521282,  0.12155956,  0.51675058], rot=[ 0.70566272, -0.01931559, -0.02865676, -0.70770462]).h_mat
pnp3=Transform(pos=[-0.06512591,  0.07280269,  0.5205142 ], rot=[ 0.70537999, -0.00925813, -0.01813629, -0.70853682]).h_mat
pnp4=Transform(pos=[-0.06515182,  0.02376428,  0.5236982 ], rot=[ 0.70521746, -0.00172719, -0.0090637 , -0.70893103]).h_mat
pnp5=Transform(pos=[-0.06522999, -0.02478053,  0.5246252 ], rot=[ 0.70496987,  0.00444041, -0.00408304, -0.7092116 ]).h_mat

# #calib
# pnp1=Transform(pos=[-0.08732946,  0.17429798,  1.33589955], rot=[ 0.70462908, -0.03195741, -0.03994709, -0.70772934]).h_mat
# pnp2=Transform(pos=[-0.08716394,  0.13364991,  1.32376706], rot=[ 0.70417846, -0.03454003, -0.04805033, -0.70755272]).h_mat
# pnp3=Transform(pos=[-0.08721339,  0.08502823,  1.32607381], rot=[ 0.70407261, -0.03399488, -0.04872422, -0.70763837]).h_mat
# pnp4=Transform(pos=[-0.08736515,  0.03632324,  1.32895535], rot=[ 0.70412823, -0.03511124, -0.04876907, -0.70752541]).h_mat
# pnp5=Transform(pos=[-0.08749534, -0.01175065,  1.33029166], rot=[ 0.70411684, -0.03485275, -0.0500498,  -0.70746009]).h_mat




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