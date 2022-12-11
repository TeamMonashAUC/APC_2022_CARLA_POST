
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy
import matplotlib
'''
using opencv2 with ROS
https://dabit-industries.github.io/turtlebot2-tutorials/14b-OpenCV2_Python.html

lane detection tutorial
https://automaticaddison.com/the-ultimate-guide-to-real-time-lane-detection-using-opencv/
'''

bridge = CvBridge()

def show_image(img,edge):
    cv2.imshow("Image Window",img)
    cv2.imshow("Edge",edge)
    cv2.waitKey(1)


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)
    
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        img_gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 
        # Sobel Edge Detection
        sobelx = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=5) # Sobel Edge Detection on the X axis
        sobely = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=5) # Sobel Edge Detection on the Y axis
        sobelxy = cv2.Sobel(src=img_blur, ddepth=cv2.CV_64F, dx=1, dy=1, ksize=5) # Combined X and Y Sobel Edge Detection
        # Display Sobel Edge Detection Images
        # cv2.imshow('Sobel X', sobelx)
        # cv2.waitKey(0)
        # cv2.imshow('Sobel Y', sobely)
        # cv2.waitKey(0)
        # cv2.imshow('Sobel X Y using Sobel() function', sobelxy)
        # cv2.waitKey(0)

        # Canny Edge Detection
        edges = cv2.Canny(image=img_blur, threshold1=100, threshold2=200) # Canny Edge Detection
        # Display Canny Edge Detection Image



    except CvBridgeError:
        rospy.logerr("CvBridge Error: {0}")

    show_image(cv_image,edges )
    # cv2.imshow("Image Window",cv_image)
    # cv2.imshow('Canny Edge Detection', edges)
    # cv2.waitKey(0)
    # show_image(cv_image)
    # show_image(gray_image)





rospy.init_node('opencv_example',anonymous=True)
rospy.loginfo("Hello ROS!")




while not rospy.is_shutdown():
    sub_image = rospy.Subscriber("/carla/ego_vehicle/rgb_top/image", Image, image_callback)
    rospy.spin()


























