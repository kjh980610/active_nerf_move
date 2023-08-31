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

pytamp= Transform(pos=[3.05935902e-01, 8.62001889e-05, 4.90078244e-01],
                            rot=[2.31976144e-04, -9.99979620e-01, -3.00290610e-04,  6.37298772e-03]).h_mat

rviz= Transform(pos=[0.30582, 9.4487e-05, 0.48332],
                            rot=[-0.00023032, 0.99998, 5.1167e-05, -0.0061789]).h_mat

py2riv = (np.linalg.inv(pytamp)).dot(rviz)    


# tcp2cam=Transform(pos=[0.05, 0.0175, -0.0532],
#                   rot=[2.5674e-16, -0.707105, 1.1102e-16, 0.707109]).h_mat
# # 생각했던값 [[0,0,-1],[0,1,0],[1,0,0]]
# # 실제 적용  [[0,0,-1],[0,-1,0],[-1,0,0]]

tcp2cam=Transform(pos=[0.05, 0.0325, -0.0534],
                  rot=[ 2.2369e-06, -0.70711, 0.70711, -2.2369e-06]).h_mat

cam2tcp=np.linalg.inv(tcp2cam)


tr = [[ 0, 0, 1, 0],
      [ 1, 0, 0, 0],
      [ 0, 1, 0, 0],
      [ 0, 0, 0, 1]]

# tcp2cam=tcp2cam.dot(tr)
# tcp2cam=py2riv.dot(tcp2cam)


step1 = Transform(
    pos=[2.30928146e-01, -1.24402053e-03,  4.85493775e-01],
    rot=[3.20609400e-03,  9.92132375e-01, -8.42289062e-04,  1.25149358e-01] #wxyz
).h_mat

step2 = Transform(
    pos=[0.3574068,  -0.12222301,  0.48665435],
    rot=[0.09463698, -0.70379775, -0.69773607,  -0.09421754]
).h_mat

step3 = Transform(
    pos=[4.82948942e-01, 4.19745683e-05,  4.86139506e-01],
    rot=[1.27692841e-01, -4.71249070e-03, -9.91790933e-01, 4.80367011e-03]
).h_mat



cam1=np.dot(step1,tcp2cam)
cam2=np.dot(step2,tcp2cam)
cam3=np.dot(step3,tcp2cam)


# pnp1= Transform(pos=[-0.0697773,   0.01926829,  0.55172168], 
# rot=[ 0.69740927,  0.07675715,  0.07397363, -0.70870061]).h_mat
# pnp2= Transform(pos=[-0.02697822, -0.16772813,  0.50942189], 
# rot=[ 0.9900315,   0.14020386,  0.00165695, -0.01333232]).h_mat
# pnp3= Transform(pos=[ 0.14765708, -0.11586462,  0.52885714], 
# rot=[ 0.71024443,  0.07887472, -0.08898568,  0.69383945]).h_mat

###solvepnp 대신 calibrateCamera 사용 시
pnp1= Transform(pos=[-0.0993095,   0.04431628,  0.91238083], rot=[ 0.68990465,  0.14277245,  0.13041821, -0.69759494]).h_mat
pnp2= Transform(pos=[-0.05454794, -0.13189779,  0.83826379], rot=[ 0.97580931,  0.21805308, -0.01278158, -0.00925603]).h_mat
pnp3= Transform(pos=[ 0.11895076, -0.08308851,  0.87274818], rot=[ 0.69853574,  0.13219625, -0.15103143,  0.68684894]).h_mat


cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam1t3 = np.dot(np.linalg.inv(cam1),cam3)

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)
pnp3t2=(np.linalg.inv(pnp3)).dot(pnp2)
pnp1t3=(np.linalg.inv(pnp1)).dot(pnp3)

print("####2t1####")
printTr(cam2t1,pnp2t1)

print("####3t2####")
printTr(cam3t2,pnp3t2)

print("####1t3####")
printTr(cam1t3,pnp1t3)
