#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils  as tu


# tcp2cam z값 확인

def printTr(h_mat1,h_mat2):
    pos1 = tu.get_pos_mat_from_homogeneous(h_mat1)
    pos2 = tu.get_pos_mat_from_homogeneous(h_mat2)
    print("pos")
    print(f"{pos1*100} \tnorm: {np.linalg.norm(pos1)*100}")
    print(f"{pos2*100} \tnorm: {np.linalg.norm(pos2)*100}")
    print(f"{(pos2-pos1)*100} \tnorm: {np.linalg.norm(pos2-pos1)*100}")
    print("rot")
    rpy1 = tu.get_rpy_from_matrix(h_mat1)
    rpy2 = tu.get_rpy_from_matrix(h_mat2)
    print(f"{rpy1} \tnorm: {np.linalg.norm(rpy1)}")
    print(f"{rpy2} \tnorm: {np.linalg.norm(rpy2)}")
    print(f"pose error: {tu.compute_pose_error(h_mat1,h_mat2)}")
    print()
    return np.linalg.norm(pos2-pos1)*100

tr = [[ 0, 1, 0, 0],
      [ 1, 0, 0, 0],
      [ 0, 0, 1, 0],
      [ 0, 0, 0, 1]]

#step1  path length:11
tcp1 = Transform(
pos=[0.22183418591460385, 0.001517997980220845,0.48901606596179914],
rot=[0.003718917137915578, -0.9921427218874871, 0.0044164164133658245, -0.1249779353608996]).h_mat
cam1 = Transform(
pos=[0.3089621992365514, -0.1428174250978963,-0.2603087454711425],
rot=[0.08574530322812836, 0.7046734533030083, 0.698427680676778, 0.0910046265240751]).h_mat
tcp2cam = Transform(
pos=[-0.09999999999975007, 0.14999999999999997,0.7466000790849354],
rot=[-0.7071067811830095, 2.236859787382861e-06, -2.236859787008161e-06, 0.7071067811830095]).h_mat

#step2  path length:9
tcp2 = Transform(
pos=[0.25503187136329336, -0.08349508093974553,0.4894416197680858],
rot=[0.04880977058801904, -0.9330332532433387, -0.33462389273138377, -0.12285521987523225]).h_mat
cam2 = Transform(
pos=[0.42268295259065136, -0.13104422436885974,-0.2585850321715611],
rot=[0.05235937786456593, 0.42313915111210354, 0.8963685801433126, 0.1213883144195755]).h_mat
tcp2cam = Transform(
pos=[-0.0999999999997496, 0.14999999999999974,0.7466000790849355],
rot=[-0.7071067811830092, 2.2368597872163276e-06, -2.236859787257961e-06, 0.7071067811830094]).h_mat

#step3  path length:9
tcp3 = Transform(
pos=[0.33474611427279477, -0.12834187345984255,0.49004867499351323],
rot=[0.08617374078483919, -0.762492884138015, -0.6330636922995433, -0.10202474959591047]).h_mat
cam3 = Transform(
pos=[0.4991328687901605, -0.05610959141645658,-0.25672516658078476],
rot=[0.011208645333854586, 0.09152022377553576, 0.9868070977499076, 0.13307955042102013]).h_mat
tcp2cam = Transform(
pos=[-0.09999999999974996, 0.14999999999999974,0.7466000790849361],
rot=[-0.7071067811830098, 2.2368597871469387e-06, -2.236859787257961e-06, 0.7071067811830098]).h_mat

#step4  path length:9
tcp4 = Transform(
pos=[0.4246309570650654, -0.1118355128196403,0.4905769237363118],
rot=[0.1132300809443029, -0.4993506667448987, -0.8562838897396152, -0.0678657539792076]).h_mat
cam4 = Transform(
pos=[0.50860406935566, 0.04919830935821021,-0.25569936733597326],
rot=[-0.03207822163039521, -0.2523898009354325, 0.9585779826102305, 0.12805712528283894]).h_mat
tcp2cam = Transform(
pos=[-0.09999999999974962, 0.1500000000000003,0.7466000790849359],
rot=[-0.7071067811830096, 2.2368597874938834e-06, -2.23685978720245e-06, 0.7071067811830095]).h_mat


cam1 = cam1.dot(tr)
cam2 = cam2.dot(tr)
cam3 = cam3.dot(tr)
cam4 = cam4.dot(tr)

pnp1=Transform( pos=[-0.08250168222452416, 0.010468500358901704,0.5572039010550308],rot=[0.6923378471587194, 0.0846170337297166, 0.066139726979269, -0.713536123479058]).h_mat
pnp2=Transform( pos=[-0.08642690894225577, -0.08439036332616298,0.5364529851896593],rot=[0.8911028139473943, 0.1251826771659694, 0.03851724445934295, -0.4344899241545831]).h_mat
pnp3=Transform( pos=[-0.02903148552229745, -0.15462488660521523,0.523694698720994],rot=[0.9847337383707232, 0.139328168071462, 0.0095783148384566, -0.10390082762483588]).h_mat
pnp4=Transform( pos=[0.06033519534371788, -0.17250168397079763,0.52461314858965],rot=[0.9613936057136431, 0.12951195519586736, -0.023733203613138774, 0.2416313791719139]).h_mat

cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam4t3 = np.dot(np.linalg.inv(cam4),cam3)
cam1t4 = np.dot(np.linalg.inv(cam1),cam4)

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)
pnp3t2=(np.linalg.inv(pnp3)).dot(pnp2)
pnp4t3=(np.linalg.inv(pnp4)).dot(pnp3)
pnp1t4=(np.linalg.inv(pnp1)).dot(pnp4)

printTr(pnp2t1,cam2t1)
printTr(pnp3t2,cam3t2)
printTr(pnp4t3,cam4t3)
printTr(pnp1t4,cam1t4)

tr = [[ 0, 1, 0, 0],
    [ 1, 0, 0, 0],
    [ 0, 0, 1, 0],
    [ 0, 0, 0, 1]]

tcp2cam = Transform(
    pos=[-0.1, 0.15,0.75],
    rot=[-0.7071067811830096, 0, 0, 0.7071067811830093]).h_mat

tr = tcp2cam.dot(tr)

t2cp1 = tcp1.dot(tr)
t2cp2 = tcp2.dot(tr)
t2cp3 = tcp3.dot(tr)
t2cp4 = tcp4.dot(tr)

tcp2t1 = np.dot(np.linalg.inv(t2cp2),t2cp1)
tcp3t2 = np.dot(np.linalg.inv(t2cp3),t2cp2)
tcp4t3 = np.dot(np.linalg.inv(t2cp4),t2cp3)
tcp1t4 = np.dot(np.linalg.inv(t2cp1),t2cp4)

# sum = 0
# sum = sum + printTr(pnp2t1,tcp2t1)
# sum = sum + printTr(pnp3t2,tcp3t2)
# sum = sum + printTr(pnp4t3,tcp4t3)
# sum = sum + printTr(pnp1t4,tcp1t4)
# # print(tcp2cam[2,3])
# print(sum)

