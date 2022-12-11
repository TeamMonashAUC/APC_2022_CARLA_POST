
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError
'''
using opencv2 with ROS
https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html
'''

bridge = CvBridge()

def show_image(img):
    cv2.imshow("Image Window",img)
    cv2.waitKey(3)


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError:
        rospy.logerr("CvBridge Error: {0}")

    show_image(cv_image)





rospy.init_node('opencv_example',anonymous=True)
rospy.loginfo("Hello ROS!")




while not rospy.is_shutdown():
    # sub_image = rospy.Subscriber("/carla/ego_vehicle/rgb_top/image", Image, image_callback)
    sub_image = rospy.Subscriber("/carla/ego_vehicle/rgb_front/image", Image, image_callback)
    rospy.spin()


























