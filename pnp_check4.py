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


tr = [[ 0, 1, 0, 0],
      [ 1, 0, 0, 0],
      [ 0, 0, -1, 0],
      [ 0, 0, 0, 1]]

cam1 = Transform(
    pos=[0.26812906325209585, -0.03243323441517007,0.550475602300727],
    rot=[0.08701690997529098, 0.7020249025834993, 0.7013220099520504, 0.08795755730917473]
).h_mat
cam2 = Transform(
    pos=[0.38899856474719957, -0.08836117690179299,0.5515156420051553],
    rot=[-0.005501658863538754, 0.0003904481389546133, 0.991586217134063, 0.12933040357843079]
).h_mat
cam3 = Transform(
    pos=[0.44526364701988397, 0.03308880170570719,0.5533326238825133],
    rot=[0.09492912639074597, 0.7008383145496726, -0.7009401423326521, -0.0921793615108393]
).h_mat

cam1 = cam1.dot(tr)
cam2 = cam2.dot(tr)
cam3 = cam3.dot(tr)


pnp1= Transform(pos=[-0.0697773,   0.01926829,  0.55172168], 
rot=[ 0.69740927,  0.07675715,  0.07397363, -0.70870061]).h_mat
pnp2= Transform(pos=[-0.02697822, -0.16772813,  0.50942189], 
rot=[ 0.9900315,   0.14020386,  0.00165695, -0.01333232]).h_mat
pnp3= Transform(pos=[ 0.14765708, -0.11586462,  0.52885714], 
rot=[ 0.71024443,  0.07887472, -0.08898568,  0.69383945]).h_mat

###solvepnp 대신 calibrateCamera 사용 시
# pnp1= Transform(pos=[-0.0993095,   0.04431628,  0.91238083], rot=[ 0.68990465,  0.14277245,  0.13041821, -0.69759494]).h_mat
# pnp2= Transform(pos=[-0.05454794, -0.13189779,  0.83826379], rot=[ 0.97580931,  0.21805308, -0.01278158, -0.00925603]).h_mat
# pnp3= Transform(pos=[ 0.11895076, -0.08308851,  0.87274818], rot=[ 0.69853574,  0.13219625, -0.15103143,  0.68684894]).h_mat


cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam1t3 = np.dot(np.linalg.inv(cam1),cam3)

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)
pnp3t2=(np.linalg.inv(pnp3)).dot(pnp2)
pnp1t3=(np.linalg.inv(pnp1)).dot(pnp3)



tcp1 = Transform(
pos=[0.4058899434216342, 9.447308480185361e-05,0.4804048842196293],
rot=[0.00018942661042711201, -0.9999564585821407, -9.250294836625947e-05, 0.009329335486911535]).h_mat
cam1 = Transform(
pos=[0.45688346651875333, -0.032416235095993055,0.5328502463547777],
rot=[-0.006728544672021098, 0.707010604581246, 0.707141422657943, -0.006460654576649662]).h_mat

tcp2 = Transform(
pos=[0.3054276694982955, 0.0001319517713470514,0.48452551565995683],
rot=[0.00022367463342259876, -0.999989490604646, -2.8258493403903115e-05, 0.004579066649056068]).h_mat
cam2 = Transform(
pos=[0.3559163819712999, -0.0323889910278106,0.5374507435388456],
rot=[-0.0033938141560393028, 0.7070793788890357, 0.7071193414330018, -0.0030774903294753992]).h_mat


# cam2 = Transform(
# pos=[0.35688346651875333, -0.032416235095993055,0.5328502463547777],
# rot=[-0.006728544672021098, 0.707010604581246, 0.707141422657943, -0.006460654576649662]).h_mat


tcp2cam = Transform(
pos=[0.05000000000025007, 0.032500000000000036,-0.05339992091506385],
rot=[-0.7071067811830093, 2.2368597873195437e-06, -2.2368597873334214e-06, 0.707106781183009]).h_mat


tcp2t1=(np.linalg.inv(tcp2)).dot(tcp1)
cam2t1=(np.linalg.inv(cam2)).dot(cam1)

# tcp = np.linalg.inv(tcp2cam).dot(tcp2t1).dot(tcp2cam)

printTr(tcp2t1,cam2t1)