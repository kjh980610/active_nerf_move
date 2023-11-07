import numpy as np
import geometry_msgs.msg
import rospy
import tf
from tf.transformations import quaternion_from_matrix
from tf.transformations import translation_from_matrix
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_matrix
from tf.transformations import translation_matrix

import image_saver

import control

test= control.control()

target_frame = "panda_hand"
source_frame = "camera_color_frame" 
listener = tf.TransformListener()
listener.waitForTransform(target_frame, source_frame,
                            rospy.Time(), rospy.Duration(4.0))

(cam_to_eef_trans, cam_to_eef_quat) = listener.lookupTransform(target_frame,
                                        source_frame,
                                        rospy.Time(0))

cam_to_eef_mat_rot = quaternion_matrix(cam_to_eef_quat)
cam_to_eef_mat_trans = translation_matrix(cam_to_eef_trans)
cam_to_eef_mat = np.matmul(cam_to_eef_mat_rot, cam_to_eef_mat_trans)



num = 5
for i in range(num):
    target_frame = "panda_link0"
    source_frame = f"{i}_pose"
    listener = tf.TransformListener()
    listener.waitForTransform(target_frame, source_frame,
                                rospy.Time(), rospy.Duration(4.0))

    (base_to_cam_trans, base_to_cam_quat) = listener.lookupTransform(target_frame,
                                            source_frame,
                                            rospy.Time(0))

    base_to_cam_mat_rot = quaternion_matrix(base_to_cam_quat)
    base_to_cam_mat_trans = translation_matrix(base_to_cam_trans)
    base_to_cam_mat = np.matmul(base_to_cam_mat_trans, base_to_cam_mat_rot)

    base_to_eef_mat = np.dot(base_to_cam_mat,cam_to_eef_mat)
    base_to_eef_trans, base_to_eef_quat = translation_from_matrix(base_to_eef_mat), quaternion_from_matrix(base_to_eef_mat)
    z_axis_flip = np.array([ 0, 0, 1, 0]) # xyzw
    base_to_eef_quat = quaternion_multiply(base_to_eef_quat, z_axis_flip)

    pose_goal = test.cal_target_pose(pos=base_to_eef_trans, rot=base_to_eef_quat)
    if test.go_to_pose_goal(pose_goal):
        image_saver.ImageSaver(i)

# br = tf.TransformBroadcaster()
# # while not rospy.is_shutdown():
    
# name = f"base_to_desired_eef"

# br.sendTransform(base_to_eef_trans,
#                     base_to_eef_quat,
#                 rospy.Time.now(),
#                 name,
#                 "panda_link0")
    

