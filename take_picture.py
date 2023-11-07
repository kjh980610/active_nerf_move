import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import message_filters

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
        
if __name__ == "__main__":

    rospy.init_node("test", anonymous=True)
    image_converter = ImageConverter()
    print(image_converter.image_sub.callback)
    
    try:
        rospy.spin()
    except:
        print("Shutdown")