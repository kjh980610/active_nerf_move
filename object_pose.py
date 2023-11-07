import numpy as np
import rospy
import tf
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('test', anonymous=True)


# x axis -> red
# y axis -> green
# z axis -> blue
# xyz -> rgb

target_frame = "panda_hand"
source_frame = "0_pose"
listener = tf.TransformListener()

listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))

(trans, rot) = listener.lookupTransform(target_frame,
                                        source_frame,
                                        rospy.Time(0))

print(trans)
rot_mat = quaternion_matrix(rot)
print(rot_mat)

desired_radius = np.linalg.norm(trans)
print(desired_radius)