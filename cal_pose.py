#! /usr/bin/env python3

import rospy
import tf
from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils as tu
import numpy as np
import math
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.4f}".format(x)})
import geometry_msgs.msg

msg_pose = geometry_msgs.msg.Pose()
pykin_pose = Transform()
# -0.70082; 0.71303; -0.020864; -0.0018597
# -0.67177; 0.68967; -0.20017; 0.18172
print(tu.get_rpy_from_quaternion([0.18172,-0.67177, 0.68967, -0.20017]))


panda_hand_to_camera_color_frame = Transform(
            pos=[0.04147810144921763, -0.04429576589937319,0.03795743837848484],
            rot=[0.7009038903068452,-0.0017877762181792646, 0.02117626918199301, 0.7129390658703675]).h_mat

cam_pose = Transform(pos=[0.34848, 0.044412, 0.55198], rot=[-3.11442877, -0.03190127, -1.58850045])


test_pose = np.dot(cam_pose.h_mat, tu.get_inverse_homogeneous(panda_hand_to_camera_color_frame))
print(tu.get_pose_from_homogeneous(test_pose))

#  0.02905, 0.95604, -0.018084, 0.29123  [ 3.08732734 -0.59186016 -0.02127827]


cal_quat = np.round(tu.get_quaternion_from_rpy([3.141592,-math.pi/6,0]),5)
# print(f"[ {cal_quat[1]}, {cal_quat[2]}, {cal_quat[3]}, {cal_quat[0]}] ")


print(tu.get_rpy_from_quaternion([0,-1,0,0]))