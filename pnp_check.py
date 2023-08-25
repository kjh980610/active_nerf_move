#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math

from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils  as tu



def printTr(h_mat1,h_mat2):
    print("pos")
    print(tu.get_pos_mat_from_homogeneous(h_mat1))
    print(tu.get_pos_mat_from_homogeneous(h_mat2))
    print("rot")
    print(tu.get_rpy_from_matrix(h_mat1))
    print(tu.get_rpy_from_matrix(h_mat2))

# cam_link1 = Transform(pos = [0.3181, -0.01755, 0.55012],
#         rot =[0.78901, 0.00016252, -0.61438, -0.00095321]).h_mat

# hand_tcp1= Transform(pos = [0.28273, 9.1909e-05, 0.48613],
#         rot=[0.99235, -0.0005591, 0.12347, -0.00078894]).h_mat

# cam_link2 = Transform(pos = [0.42401, -0.088348, 0.55143],
#         rot =[0.55797, 0.5631, -0.43412, 0.42794]).h_mat

# hand_tcp2= Transform(pos = [0.40579, -0.12296, 0.48718],
#         rot=[0.70152, 0.70077, 0.087569, -0.095569]).h_mat


# print(np.linalg.inv(cam_link1).dot(hand_tcp1))
# print(np.linalg.inv(cam_link2).dot(hand_tcp2))





# pose1 = Transform(pos = [0.35689057, -0.15, 1.60378205],
#         rot =[0.09264647, -0.70101115, -0.70110344, -0.09194542]).h_mat

# pose2 = Transform(pos = [ 2.06890567e-01,  -5.21846463e-12, 1.60378205e+00],
#         rot= [4.95722410e-04, -9.91444737e-01, -6.52630936e-05, -1.30526176e-01]).h_mat


# pnp1 = Transform(pos=[ 5.80335672,  1.3429377,  27.32000169],
#         rot=[ 2.58693510e-03, -7.39660309e-04,  1.38344824e-01, -9.90380468e-01]).h_mat

# pnp2 = Transform(pos=[ 6.82880257, -6.89614822, 24.99778682],
#         rot=[ 0.6939018, 0.07186997, -0.08365384, 0.71157363]).h_mat

# # print(tu.get_inverse_homogeneous(pose1))
# print(pnp2.dot(tu.get_inverse_homogeneous(pnp1)))


# pnp1 =Transform(pos=[ 5.80335672,  1.3429377,  27.32000169],
# rot=[ 2.58693510e-03, -7.39660309e-04,  1.38344824e-01, -9.90380468e-01]).h_mat

# pnp2 = Transform(pos=[ 6.53495361, -6.89506916, 25.07079668], 
# rot=[ 0.69250464,  0.06731192, -0.08691179,  0.71298862]).h_mat

# print(pnp1.dot(tu.get_inverse_homogeneous(pnp2)))

# print(pose2.dot(tu.get_inverse_homogeneous(pose1)))

# t_a = Transform(pos=[0.31810159995555426, -0.01747697170262794, 0.5501198615743758],
# rot = [-0.7890040066973754, -0.0002560878151948469, 0.6143873685597049, 0.0008793115490433178]).h_mat

# t_b = Transform(pos=[0.42400718064171794, -0.0883496632840712,0.5514306983112311],
#         rot=[-0.5579671054444647, -0.5630976024289456, 0.43412562549682376, -0.42793543983607096]).h_mat


# pnp_a = Transform(pos=[ 6.53495361, -6.89506916, 25.07079668], 
# rot=[ 0.69250464,  0.06731192, -0.08691179,  0.71298862]).h_mat

# pnp_b =Transform(pos=[ 5.80335672,  1.3429377,  27.32000169],
# rot=[ 2.58693510e-03, -7.39660309e-04,  1.38344824e-01, -9.90380468e-01]).h_mat


# t_ba = tu.get_inverse_homogeneous(t_b).dot(t_a)


# pnp_ba = tu.get_inverse_homogeneous(pnp_b).dot(pnp_a)


# # print(pnp_a)
# # print(pnp_b.dot(pnp_ba))


# # print(t_ba)
# # print(pnp_ba)
# # print(tu.get_inverse_homogeneous(pnp_ba))


# tcp_a=Transform(pos=[0.28272223124614765, 9.20175287190766e-05,0.48613099757080735],
# rot=[-0.9923472145717677, 0.0005597494327553489, -0.12347495189172547, 0.0007928855920561558]).h_mat

# tcp_b = Transform(pos=[0.4057903796899095, -0.12296583604640934,0.48718375052278995],
# rot=[-0.7015157746270342, -0.7007664869007744, -0.0875669310903252, 0.095571864926878]).h_mat

# tcp_ba = tu.get_inverse_homogeneous(tcp_a).dot(tcp_b)
# print(tcp_ba)



# t_a = Transform( #tcp
# pos=[0.28272346771459395, 0.00016014169036416035,0.48612926815368473],
# rot=[-0.9923476337327072, 0.0004397412879637363, -0.1234719814548614, 0.000806379753460609]).h_mat
# tcp_a = Transform( #camera
# pos=[0.31810121114639667, -0.017473086359913976,0.550123690504078],
# rot=[-0.7890055600291527, -0.00025924975779699516, 0.6143853697984483, 0.0008811414586848945]).h_mat
# bb = Transform(
# pos=[0.05000000000025012, 0.01750000000000007,-0.05339992091506385],
# rot=[4.163336342344337e-17, -0.7071045443232222, 0.0, 0.707109018042797]).h_mat

# # print(tu.get_inverse_homogeneous(tcp_a).dot(t_a))
# # print(tu.get_inverse_homogeneous(tcp_b).dot(t_b))
# # print(bb)

# t_b = Transform(
# pos=[0.40578780191024394, -0.1229675201682202,0.48718837782303753],
# rot=[-0.7015140664652962, -0.70076774336637, -0.08757266751239924, 0.09556993414925172]).h_mat
# tcp_b = Transform(
# pos=[0.42400378303619235, -0.08835164714659768,0.5514357195604984],
# rot=[-0.557969953838009, -0.5630971256254675, 0.4341203613721929, -0.42793769355879757]).h_mat
# bb2 = Transform(
# pos=[0.05000000000025023, 0.01750000000000006,-0.0533999209150639],
# rot=[-1.5207290721824807e-16, -0.7071045443232217, -5.072981965059675e-16, 0.7071090180427968]).h_mat


# print(tu.get_rpy_from_matrix(bb))
# # print(tu.get_rpy_from_matrix(bb2))

# print(tu.get_pos_mat_from_homogeneous(bb))
# # print(tu.get_pos_mat_from_homogeneous(bb2))

# # print(tu.get_rpy_from_matrix(tu.get_inverse_homogeneous(tcp_a).dot(t_a)))
# # print(tu.get_rpy_from_matrix(tu.get_inverse_homogeneous(tcp_b).dot(t_b)))   #결론! 계산 방식이 이상하다!!


# print(t_a.dot(bb))
# print(tcp_a)


step1 = np.array([
 [ 9.69511652e-01, -1.45333070e-04,  2.45045172e-01,  2.80999319e-01],
 [-5.01809634e-04, -9.99998905e-01,  1.39230401e-03,  1.98590530e-04],
 [ 2.45044701e-01, -1.47282099e-03, -9.69510663e-01,  4.92812368e-01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

step2 = np.array([
 [ 2.13791691e-03,  9.99934147e-01, -1.12752007e-02,  4.05880686e-01],
 [ 9.66479807e-01,  8.28610778e-04,  2.56741304e-01, -1.24767232e-01],
 [ 2.56733739e-01, -1.14461454e-02, -9.66414390e-01,  4.93760105e-01],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

tcp2cam=Transform(pos=[0.05, 0.0175, -0.0534],
                  rot=[2.5674e-16, -0.707105, 1.1102e-16, 0.707109]).h_mat
cam2tcp=np.linalg.inv(tcp2cam)


cam1=np.dot(step1,tcp2cam)
cam2=np.dot(step2,tcp2cam)

step2t1= np.dot(np.linalg.inv(step2),step1)

cam2t1 = np.dot(np.linalg.inv(cam2),cam1)

pnp1 = Transform(pos=[ 6.53495361, -6.89506916, 25.07079668], 
rot=[ 0.69250464,  0.06731192, -0.08691179,  0.71298862]).h_mat


pnp2 =Transform(pos=[ 5.80335672,  1.3429377,  27.32000169],
rot=[ 2.58693510e-03, -7.39660309e-04,  1.38344824e-01, -9.90380468e-01]).h_mat

pnp2t1=(np.linalg.inv(pnp2)).dot(pnp1)

printTr(cam2t1,pnp2t1)