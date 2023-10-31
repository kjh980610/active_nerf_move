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
    return np.linalg.norm(pos2-pos1)*100

tr = [[ 0, 1, 0, 0],
      [ 1, 0, 0, 0],
      [ 0, 0, 1, 0],
      [ 0, 0, 0, 1]]

#step1  path length:61
tcp1 = Transform(
pos=[0.2123731226627255, -0.001317922558528858,0.48862543388147206],
rot=[-0.003774470059462267, -0.9922468829545968, -0.004886976595044014, -0.12412894143758993]).h_mat
cam1 = Transform(
pos=[0.10351172723138355, -0.15173255235790123,0.5174712767201025],
rot=[-0.08664211262135448, -0.7033577740791811, -0.7002493262041559, -0.08620831007026805]).h_mat

# #step2  path length:9
# tcp2 = Transform(
# pos=[0.24044776208022234, -0.08198272865177991,0.4888948763506552],
# rot=[0.04510652783085968, -0.9443405744126627, -0.3020659611081337, -0.1222392563983673]).h_mat
# cam2 = Transform(
# pos=[0.23778226457482068, -0.2682629988271414,0.5133898347799967],
# rot=[-0.050513372858368155, -0.46051282127421933, -0.8782967785493874, -0.11819947125920734]).h_mat

# #step3  path length:9
# tcp3 = Transform(
# pos=[0.3134112793507541, -0.13371093620546465,0.48985550458070437],
# rot=[0.07941570011164564, -0.8048189846547648, -0.578352176538575, -0.10709018819734956]).h_mat
# cam3 = Transform(
# pos=[0.4201779965533738, -0.2863155205097976,0.5147584335066302],
# rot=[-0.01576053138038161, -0.16712050291213945, -0.9771288084399421, -0.13054363441162944]).h_mat

#step4  path length:9
tcp2 = Transform(
pos=[0.4018761928643996, -0.13287358751749082,0.4901366076392898],
rot=[0.10720891549262784, -0.5861871330826229, -0.7992185174873003, -0.07836232993391415]).h_mat
cam2 = Transform(
pos=[0.5779150031432827, -0.1934143194343077,0.5156827223455165],
rot=[0.02360340692583151, 0.14370513262089335, -0.9809213864249116, -0.12878258153100858]).h_mat

#step5  path length:9
tcp3 = Transform(
pos=[0.4740484653327811, -0.07971783459039755,0.4906031538935697],
rot=[0.1226614699089469, -0.31055827314686657, -0.9416266085248555, -0.04297735331300784]).h_mat
cam3 = Transform(
pos=[0.6518778710523006, -0.025334556368835803,0.5175588292390497],
rot=[0.05863146349237245, 0.44002757533035197, -0.8888070495247453, -0.11384251738435856]).h_mat

cam1 = cam1.dot(tr)
cam2 = cam2.dot(tr)
cam3 = cam3.dot(tr)


pnp1=Transform( pos=[-0.10969111478166323, 0.012725198780837227,0.5596827839320347],rot=[0.705407875404985, 0.07925143229495932, 0.07301699716107717, -0.7005622441449453]).h_mat
pnp2=Transform( pos=[0.04801254356029333, -0.1944421305577059,0.5165432100015404],rot=[0.9796171156989897, 0.13534456935889805, -0.014114515198076244, 0.14775971925856546]).h_mat
pnp3=Transform( pos=[0.14272311892102385, -0.1608100533589841,0.5298055132344911],rot=[0.890124135715502, 0.1141368823309625, -0.04638795173634251, 0.43874816585622933]).h_mat

cam2t1 = np.dot(np.linalg.inv(cam2),cam1)
cam3t2 = np.dot(np.linalg.inv(cam3),cam2)
cam1t3 = np.dot(np.linalg.inv(cam1),cam3)

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)
pnp3t2=(np.linalg.inv(pnp3)).dot(pnp2)
pnp1t3=(np.linalg.inv(pnp1)).dot(pnp3)

# printTr(pnp2t1,cam2t1)
# printTr(pnp3t2,cam3t2)
# printTr(pnp1t3,cam1t3)


tr = [[ 0, 1, 0, 0],
    [ 1, 0, 0, 0],
    [ 0, 0, 1, 0],
    [ 0, 0, 0, 1]]

tcp2cam = Transform(
    pos=[-0.1, 0.15,-0.0534],
    rot=[-0.7071067811830096, 0, 0, 0.7071067811830093]).h_mat

tr = tcp2cam.dot(tr)


t2cp1 = tcp1.dot(tr)
t2cp2 = tcp2.dot(tr)
t2cp3 = tcp3.dot(tr)



tcp2t1 = np.dot(np.linalg.inv(t2cp2),t2cp1)
tcp3t2 = np.dot(np.linalg.inv(t2cp3),t2cp2)
tcp1t3 = np.dot(np.linalg.inv(t2cp1),t2cp3)

sum = 0
sum = sum + printTr(pnp2t1,tcp2t1)
sum = sum + printTr(pnp3t2,tcp3t2)
sum = sum + printTr(pnp1t3,tcp1t3)
# print(tcp2cam[2,3])
# print(sum)

