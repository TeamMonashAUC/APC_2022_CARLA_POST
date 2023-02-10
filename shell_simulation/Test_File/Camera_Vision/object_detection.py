# Install YoloV5 before running: https://github.com/ultralytics/yolov5
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import torch
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.cuda() # use GPU
print(model.names[0])
bridge = CvBridge()

def calculate_boxArea(x1,y1,x2,y2):
    width = x2 - x1
    height = y2 - y1
    area = width * height
    return area

def show_image(img,output):
    try:
        # for each object detected, draw a bounding box
        for i in range(len(output.xyxy)):
            for j in range(0,3):
                # Get the coordinates of the bounding box
                x1, y1, x2, y2, probability, index = output.xyxy[i][j]
                x1 = int(x1)
                y1 = int(y1)
                x2 = int(x2)
                y2 = int(y2)
                area = calculate_boxArea(x1,y1,x2,y2)
                # Get the object label and probability
                label = model.names[int(index)]
                probability = probability * 100
                colour = (255,0,0) # (B,G,R)
                # if the area or person crosses threshold then warn user
                if area > 50000 and probability > 50 or (int(index) == 0 and area > 10000):
                    colour = (0,0,255)
                    text = f"Too Close: {label}"
                    cv2.putText(img, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
                    # Draw the bounding box on the image                 
                    cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)

                elif probability > 50:
                    # Display the object label and probability
                    text = f"{label}: {probability:.2f}%"
                    cv2.putText(img, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
                    # Draw the bounding box on the image                 
                    cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
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
