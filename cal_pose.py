#! /usr/bin/env python3

import rospy
import tf
from pykin.kinematics.transform import Transform
import pykin.utils.transform_utils as tu
import numpy as np
import math

import geometry_msgs.msg

msg_pose = geometry_msgs.msg.Pose()
pykin_pose = Transform()

print(tu.get_rpy_from_quaternion([0.02905, 0.95604, -0.018084, 0.29123]))

#  0.02905, 0.95604, -0.018084, 0.29123  [ 3.08732734 -0.59186016 -0.02127827]


cal_quat = np.round(tu.get_quaternion_from_rpy([3.141592,-math.pi/6,0]),5)
print(f"[ {cal_quat[1]}, {cal_quat[2]}, {cal_quat[3]}, {cal_quat[0]}] ")


msg_pose.orientation = [1,0,0,0]
# print(pykince)