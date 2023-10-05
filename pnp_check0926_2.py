#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils  as tu


#<origin xyz="-0.125 0.1175 0.01" rpy="0 -1.57079 0"/> 이상하게 해놓고 결과 보기

def printTr(h_mat1,h_mat2):
    print("pos")
    pos1 = tu.get_pos_mat_from_homogeneous(h_mat1)
    pos2 = tu.get_pos_mat_from_homogeneous(h_mat2)
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


tr = [[ 0, 1, 0, 0],
      [ 1, 0, 0, 0],
      [ 0, 0, 1, 0],
      [ 0, 0, 0, 1]]

tcp1 = Transform( #0
pos=[0.3055619885210481, -5.679761889484026e-05,0.4841376574667514],
rot=[-0.0011437847935383669, -0.9999703420622122, 0.0043251186502065294, 0.006268979262243954]).h_mat
cam1 = Transform(
pos=[0.2049482783441064, -0.14906510436976805,0.5391372701685974],
rot=[-0.0036218132955728143, 0.7101441420454073, 0.7040275057084154, -0.0052393886123908604]).h_mat

tcp2 = Transform( #180
pos=[0.3076895972060073, 0.0027520862070927465,0.4876976964328287],
rot=[0.0017955918768729415, 0.013199420194907663, 0.9999044871683426, 0.0036834354811689334]).h_mat
cam2 = Transform(
pos=[0.41141462796097544, 0.14966397229013467,0.5425572195449495],
rot=[-0.0038720502782725887, 0.6977058561403675, -0.7163726387167165, -0.0013371731858016066]).h_mat

tcp3 = Transform( #90
pos=[0.3079573082637799, 0.0017636837397104893,0.48640368082404595],
rot=[0.0010093622730745504, -0.7097627921607418, -0.7044382036222122, -0.0016054063349375857]).h_mat
cam3 = Transform(
pos=[0.457154398646073, -0.09955983017354735,0.5395574699967172],
rot=[0.000421478708385091, 0.0037650513293365495, 0.9999911082250209, 0.0018520839832885034]).h_mat

tcp4 = Transform( #270
pos=[0.30754479039951826, -0.0015395287882238179,0.4857699134545337],
rot=[-0.002520096121466893, -0.7097418419821149, 0.7044567994889774, -0.0008857251941142406]).h_mat
cam4 = Transform(
pos=[0.15692387758313497, 0.09759390044474596,0.5390376840532387],
rot=[0.0024114427118175205, 0.9999894417056644, 0.0037370930415966674, -0.001155662943800545]).h_mat

cam1 = cam1.dot(tr)
cam2 = cam2.dot(tr)
cam3 = cam3.dot(tr)
cam4 = cam4.dot(tr)


pnp1=Transform( pos=[-0.11144763197834068, 0.05208173995788391,0.520575237788347],rot=[0.7027217383060883, -0.0009487813134532618, -0.018961993097616574, -0.7112114320958676]).h_mat
pnp2=Transform( pos=[0.1926787333588158, -0.14979156682486428,0.5294315590051596],rot=[0.718097411839367, 0.010457694389366824, -0.009746959672592898, 0.6957957606329619]).h_mat
pnp3=Transform( pos=[-0.055711528160328025, -0.2049288458082117,0.5252681390860429],rot=[0.9998994334324359, 0.004987925871308708, -0.011918627143389683, -0.005847216774903968]).h_mat
pnp4=Transform( pos=[0.1305870485664041, 0.10245392079841857,0.5226144670239832],rot=[0.002597809457910775, -0.01322760296602492, -0.013604464559282345, -0.9998165834041006]).h_mat

cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam4t3 = np.dot(np.linalg.inv(cam4),cam3)
cam1t4 = np.dot(np.linalg.inv(cam1),cam4)
cam1t3 = np.dot(np.linalg.inv(cam1),cam3)

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)
pnp3t2=(np.linalg.inv(pnp3)).dot(pnp2)
pnp4t3=(np.linalg.inv(pnp4)).dot(pnp3)
pnp1t4=(np.linalg.inv(pnp1)).dot(pnp4)
pnp1t3=(np.linalg.inv(pnp1)).dot(pnp3)

# printTr(pnp2t1,cam2t1)
# printTr(pnp3t2,cam3t2)
# printTr(pnp4t3,cam4t3)
# printTr(pnp1t4,cam1t4)
# printTr(pnp1t3,cam1t3)


tcp2cam = Transform(
    pos=[-0.1, 0.15,-0.0534],
    rot=[-0.7071067811830096, 0, 0, 0.7071067811830093]).h_mat

tr = tcp2cam.dot(tr)


tcp1 = tcp1.dot(tr)
tcp2 = tcp2.dot(tr)
tcp3 = tcp3.dot(tr)
tcp4 = tcp4.dot(tr)

tcp2t1 = np.dot(np.linalg.inv(tcp2),tcp1)


tcp2t1 = np.dot(np.linalg.inv(tcp2),tcp1)
tcp3t2 = np.dot(np.linalg.inv(tcp3),tcp2)
tcp4t3 = np.dot(np.linalg.inv(tcp4),tcp3)
tcp1t4 = np.dot(np.linalg.inv(tcp1),tcp4)
tcp1t3 = np.dot(np.linalg.inv(tcp1),tcp3)


printTr(pnp2t1,tcp2t1)
printTr(pnp3t2,tcp3t2)
printTr(pnp4t3,tcp4t3)
printTr(pnp1t4,tcp1t4)
printTr(pnp1t3,tcp1t3)

