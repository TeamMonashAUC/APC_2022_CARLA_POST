# Install YoloV5 before running: https://github.com/ultralytics/yolov5
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import torch
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.cuda() # use GPU

bridge = CvBridge()

def show_image(img,output):
    try:
        for i in range(len(output.xyxy)):
            for j in range(0,3):
                # Get the coordinates of the bounding box
                x1, y1, x2, y2, probability, index = output.xyxy[i][j]
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                # Get the object label and probability
                label = model.names[int(index)]
                probability = probability * 100

                if probability > 50:
                    # Display the object label and probability
                    text = f"{label}: {probability:.2f}%"
                    cv2.putText(img, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                    # Draw the bounding box on the image
                    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    print(output.xyxy[0])
    except:
        pass
    cv2.imshow("Object Detection", img)
    cv2.waitKey(3)


def image_callback(img_msg):
    #rospy.loginfo(img_msg.header)
    
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError:
        rospy.logerr("CvBridge Error: {0}")

    # pass image through YOLOv5
    output = model(cv_image)
    show_image(cv_image,output)

rospy.init_node('opencv_example',anonymous=True)
rospy.loginfo("Hello ROS!")

while not rospy.is_shutdown():
    # sub_image = rospy.Subscriber("/carla/ego_vehicle/rgb_top/image", Image, image_callback)
    sub_image = rospy.Subscriber("/carla/ego_vehicle/rgb_front/image", Image, image_callback)
    rospy.spin()
