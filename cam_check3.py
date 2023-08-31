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
pos=[0.232353977894598, -0.0011466211331613442,0.4893030819632792],
rot=[-0.003360098268812312, -0.9919948833194057, -0.005205245465463873, -0.12612599505013358]).h_mat
cam1 = Transform(
pos=[0.26770430409521456, -0.03279951948069459,0.5537720668899152],
rot=[0.09156270196478201, 0.6977653548854131, 0.705126698650254, 0.08681082870895371]).h_mat

tcp2 = Transform(
pos=[0.3578098940054397, -0.12108083700534396,0.49004897624923754],
rot=[0.09349908187535, -0.7047865134908782, -0.6968919128113849, -0.09421015841154198]).h_mat
cam2 = Transform(
pos=[0.3907221062928413, -0.08725931033989556,0.5547072484974771],
rot=[0.0005028246997963309, 0.00558232408462328, 0.9911359003970084, 0.13273361205567877]).h_mat

tcp3 = Transform(
pos=[0.4813533376169339, -1.0320381101880383e-05,0.4896792403565199],
rot=[-0.12746212050460523, 0.005769418817831384, 0.9918158917884943, -0.004621519097470987]).h_mat
cam3 = Transform(
pos=[0.44681935131149897, 0.03352825400472065,0.5536352359455401],
rot=[0.09339944289366481, 0.6972398521465629, -0.7053990631610901, -0.08686365371804938]).h_mat

#pnp
pnp3=Transform(pos=[ 0.14567223, -0.13305585,  0.53453622], rot=[ 0.70135788,  0.07619617, -0.09465855,  0.70237528]).h_mat
pnp2=Transform(pos=[-0.04094114, -0.1677631,   0.51848472], rot=[ 0.99006443,  0.14059064, -0.00232104, -0.0011434 ]).h_mat
pnp1=Transform(pos=[-0.07032172,  0.02734028,  0.56504757], rot=[ 0.71121271,  0.07872411,  0.07189378, -0.6948455 ]).h_mat

# #calib
# pnp3=Transform(pos=[ 0.11598076, -0.11637627,  0.985431  ], rot=[ 0.68604384,  0.14722048, -0.17080911,  0.69173277]).h_mat
# pnp2=Transform(pos=[-0.06965,    -0.14783675,  0.95422218], rot=[ 0.97004506,  0.2423898,  -0.01587043,  0.00280914]).h_mat
# pnp1=Transform(pos=[-0.1013689,   0.03179261,  1.04320588], rot=[ 0.69947771,  0.1653929,   0.1453738,  -0.67988424]).h_mat

# tr = tu.get_h_mat(orientation=[0,0,np.pi])

# cam1 = np.dot(cam1,tr)
# cam2 = np.dot(cam2,tr)
# cam3 = np.dot(cam3,tr)

cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam1t3 = np.dot(np.linalg.inv(cam1),cam3)

tcp2t1 = np.dot(np.linalg.inv(tcp2),tcp1)
tcp3t2 = np.dot(np.linalg.inv(tcp3),tcp2)
tcp1t3 = np.dot(np.linalg.inv(tcp1),tcp3)

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)
pnp3t2=(np.linalg.inv(pnp3)).dot(pnp2)
pnp1t3=(np.linalg.inv(pnp1)).dot(pnp3)

print("####2t1####")
printTr(cam2t1,pnp2t1)
print("####3t2####")
printTr(cam3t2,pnp3t2)
print("####1t5####")
printTr(cam1t3,pnp1t3)

tr1 = tu.get_rotation_matrix([0,0,3.12])
tr2 = tu.get_rotation_matrix([0,0,-3.12])

print(tu.get_rpy_from_matrix(np.linalg.inv(tr1).dot(tr2)))

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

