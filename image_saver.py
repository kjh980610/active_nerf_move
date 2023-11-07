#!/usr/bin/env python3
# Python 2/3 compatibility imports

import rospy
import tf
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


import numpy as np

class ImageSaver:
    def __init__(self,num=0):

        self.rgb_image = None

        target_frame = "camera_color_frame"
        source_frame = "panda_link0"


        listener = tf.TransformListener()
        listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))

        (trans, rot) = listener.lookupTransform(target_frame,
                                                source_frame,
                                                rospy.Time(0))
        
        # num=29
        print(f"{num}.")
        print(f"trans : {trans}")
        print(f"rot : {rot}")

        matrix = quaternion_matrix(rot)
        matrix[:3, 3] = trans
        print(matrix)

        
        
        self.file_name = f"{num}.png"

        self.sub1 = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)

    def rgb_image_callback(self, msg):
        try:
            self.rgb_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.save_images()
        except Exception as e:
            rospy.logerr(e)

    def save_images(self):
        if self.rgb_image is not None :
            cv2.imwrite(self.file_name, self.rgb_image)
            rospy.loginfo("Images saved successfully!")

            # 이미지를 저장한 후에는 subscriber 제거
            self.sub1.unregister()

    def run(self):
        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('img_saver', anonymous=True)
    image_saver = ImageSaver()
    image_saver.run()
    exit()