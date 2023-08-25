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


tcp2cam=Transform(pos=[0.05, 0.0175, -0.0532],
                  rot=[2.5674e-16, -0.707105, 1.1102e-16, 0.707109]).h_mat

# 생각했던값 [[0,0,-1],[0,1,0],[1,0,0]]
# 실제 적용  [[0,0,-1],[0,-1,0],[-1,0,0]]

cam2tcp=np.linalg.inv(tcp2cam)


tr = [[ 0, 0, 1, 0],
      [ 1, 0, 0, 0],
      [ 0, 1, 0, 0],
      [ 0, 0, 0, 1]]

tcp2cam=tcp2cam.dot(tr)
tcp2cam=py2riv.dot(tcp2cam)


step1 = Transform(
    pos=[ 0.23095883, -0.00127291,  0.48553702],
    rot=[ 0.00349059,  0.9921077,   0.00526302,  0.12522949]
).h_mat

step2 = Transform(
    pos=[ 0.26861294, -0.08603428,  0.48625199  ],
    rot=[ 0.05388143, -0.91728809, -0.37558488, -0.12089394]
).h_mat

step3 = Transform(
    pos=[ 0.35712812, -0.12187571, 0.48675388 ],
    rot=[ 0.09498628, -0.70355782, -0.69798956, -0.09377937]
).h_mat

step4 = Transform(
    pos=[ 0.44591117, -0.08515735,  0.48701143 ],
    rot=[ 0.12081376, -0.38337641, -0.91410563, -0.05326781]
).h_mat

step5 = Transform(
    pos=[ 0.48197732,  0.00364449,  0.4869314 ],
    rot=[ 0.1292448,  -0.00501552, -0.99159466, -0.00326498]
).h_mat



cam1=np.dot(step1,tcp2cam)
cam2=np.dot(step2,tcp2cam)
cam3=np.dot(step3,tcp2cam)
cam4=np.dot(step4,tcp2cam)
cam5=np.dot(step5,tcp2cam)


#pnp
pnp1=Transform(pos=[-0.07108612,  0.01767745,  0.55156537], rot=[ 0.70161983,  0.07765068,  0.07460415, -0.70436795]).h_mat
pnp2=Transform(pos=[-0.09227032, -0.08691794,  0.52580509], rot=[ 0.9106733,   0.1270235,  0.03560433, -0.39149904]).h_mat
pnp3=Transform(pos=[-0.02839167, -0.16809629,  0.50934996], rot=[ 9.90029515e-01,  1.40281468e-01,  9.23843402e-04, -1.27206708e-02]).h_mat
pnp4=Transform(pos=[ 0.07754054, -0.18059749,  0.51144745], rot=[ 0.92061169,  0.12257157 -0.03739371,  0.36885233]).h_mat
pnp5=Transform(pos=[ 0.15878249, -0.11940038,  0.52839406], rot=[ 0.71007666,  0.08613928 -0.08384543,  0.69378751]).h_mat

# #calib
# pnp1=Transform(pos=[-0.09573168,  0.02928164,  0.98249613], rot=[ 0.69096424,  0.15507904,  0.14530468, -0.69094534]).h_mat
# pnp2=Transform(pos=[-0.11622877, -0.06820163,  0.93735675], rot=[ 0.89641696,  0.21495414,  0.06799332, -0.38158651]).h_mat
# pnp3=Transform(pos=[-0.0514537,  -0.14291157,  0.90301627], rot=[ 0.97183969,  0.23526292, -0.00975875, -0.00915065]).h_mat
# pnp4=Transform(pos=[ 0.05398299, -0.15432199,  0.90616836], rot=[ 0.90294524,  0.21100949, -0.08611759,  0.36434688]).h_mat
# pnp5=Transform(pos=[ 0.13473813, -0.09794939,  0.93900154], rot=[ 0.69615012,  0.15150703, -0.15954408,  0.68334934]).h_mat


cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam4t3 = np.dot(np.linalg.inv(cam4),cam3)
cam5t4 = np.dot(np.linalg.inv(cam5),cam4)
cam1t5 = np.dot(np.linalg.inv(cam1),cam5)

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