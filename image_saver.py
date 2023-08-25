import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from pykin.utils.transform_utils import get_pose_from_homogeneous
from pykin.kinematics.transform import Transform
import numpy as np

class ImageSaver:
    def __init__(self,pose,file_num,check_board = False):

        self.rgb_image = None
        self.depth_image = None
        pose =get_pose_from_homogeneous(pose)
        dir= "active_nerf_move/"
        if check_board:
            self.file_name = f"check_img/c{file_num} {pose}.png"
        else:
            self.file_name =  f"target_img/c{file_num} {pose}.png"

        self.file_name = dir + self.file_name

        self.sub1 = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_image_callback)

    def rgb_image_callback(self, msg):
        try:
            self.rgb_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.save_images()
        except Exception as e:
            rospy.logerr(e)

    def depth_image_callback(self, msg):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_image  = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    
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
    image_saver = ImageSaver(Transform().h_mat)
    image_saver.run()