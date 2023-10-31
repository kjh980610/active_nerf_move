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
pos=[0.21187162800781492, -0.0016006426150166995,0.4888482610929187],
rot=[-0.004217223876453208, -0.9921655314287852, -0.004617938030881229, -0.12477358661093153]).h_mat
cam1 = Transform(
pos=[0.10287661879678331, -0.15192251263109782,0.5176730817162578],
rot=[-0.0874103110783068, -0.7034897556865196, -0.700002834962536, -0.08635761586209134]).h_mat
#step2  path length:9
tcp2 = Transform(
pos=[0.24034142764646677, -0.08186357235264882,0.48874505218715963],
rot=[0.04532734886781407, -0.9442249740915437, -0.3025079465126677, -0.12195725498357196]).h_mat
cam2 = Transform(
pos=[0.23787387648019662, -0.26815001751456835,0.5132137763506517],
rot=[-0.050158326004423374, -0.46011970648841544, -0.8785295012545632, -0.1181521014223361]).h_mat
#step3  path length:9
tcp3 = Transform(
pos=[0.31347540574237215, -0.13363277014198105,0.48978479922042073],
rot=[0.07931704945909315, -0.8044927178069179, -0.5788161319926026, -0.10710816031407155]).h_mat
cam3 = Transform(
pos=[0.42040280716044465, -0.2861151738541493,0.5147466562456873],
rot=[-0.015844247679006586, -0.16656278156160179, -0.9772304330355099, -0.13048557765069227]).h_mat
#step4  path length:19
tcp4 = Transform(
pos=[0.40205007797096887, -0.1322630407444714,0.48995978014742814],
rot=[0.10718005431429412, -0.5854472889695586, -0.7997899614265753, -0.07810201915863449]).h_mat
cam4 = Transform(
pos=[0.5782043449737564, -0.1924587812726304,0.5155252639290689],
rot=[0.02376343951286841, 0.14463300070112278, -0.980808409590215, -0.1285747383410796]).h_mat
#step5  path length:9
tcp5 = Transform(
pos=[0.4741260323902329, -0.07936591748263316,0.49056883292308007],
rot=[0.12262814974676395, -0.3100932540559683, -0.9417727300490124, -0.043227718129642566]).h_mat
cam5 = Transform(
pos=[0.6518983675588025, -0.02483713424552375,0.5176068722271077],
rot=[0.058430603948818174, 0.4404613909142667, -0.8885855960685987, -0.11399680568821165]).h_mat

cam1 = cam1.dot(tr)
cam2 = cam2.dot(tr)
cam3 = cam3.dot(tr)
cam4 = cam4.dot(tr)
cam5 = cam5.dot(tr)

pnp1=Transform( pos=[-0.07370257450350162, 0.018094669481907927,0.5616775037676874],rot=[0.6991037700075807, 0.07689294647042802, 0.07251686492064706, -0.707165254976773]).h_mat
pnp2=Transform( pos=[-0.09083913274677963, -0.06345223936117972,0.5406853766221397],rot=[0.8764490654472222, 0.11986723980397596, 0.03931580872873664, -0.4646753142598306]).h_mat
pnp3=Transform( pos=[-0.04394398410351759, -0.13419282107261546,0.5285198257440679],rot=[0.975431099697992, 0.13701674985487408, 0.015951405505867145, -0.1717734923192756]).h_mat
pnp4=Transform( pos=[0.03242935735077648, -0.16425062117652736,0.5258377005944636],rot=[0.9810541318558668, 0.13347885690933728, -0.013255481366057848, 0.13978725743236534]).h_mat
pnp5=Transform( pos=[0.11354816937433124, -0.14371600037153126,0.5351889630028694],rot=[0.8908911272037461, 0.11258120721906452, -0.04542267335333516, 0.4376931025229585]).h_mat

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

# printTr(pnp2t1,cam2t1)
# printTr(pnp3t2,cam3t2)
# printTr(pnp4t3,cam4t3)
# printTr(pnp5t4,cam5t4)
# printTr(pnp1t5,cam1t5)

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
t2cp5 = tcp5.dot(tr)

tcp2t1 = np.dot(np.linalg.inv(t2cp2),t2cp1)
tcp3t2 = np.dot(np.linalg.inv(t2cp3),t2cp2)
tcp4t3 = np.dot(np.linalg.inv(t2cp4),t2cp3)
tcp5t4 = np.dot(np.linalg.inv(t2cp5),t2cp4)
tcp1t5 = np.dot(np.linalg.inv(t2cp1),t2cp5)

sum = 0
sum = sum + printTr(pnp2t1,tcp2t1)
sum = sum + printTr(pnp3t2,tcp3t2)
sum = sum + printTr(pnp4t3,tcp4t3)
sum = sum + printTr(pnp5t4,tcp5t4)
sum = sum + printTr(pnp1t5,tcp1t5)
# print(tcp2cam[2,3])
print(sum)

