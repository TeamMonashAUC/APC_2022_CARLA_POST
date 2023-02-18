# Install YoloV5 before running: https://github.com/ultralytics/yolov5
import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

import torch

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model.cuda() # use GPU
bridge = CvBridge()

def calculate_boxArea(x1,y1,x2,y2):
    width = x2 - x1
    height = y2 - y1
    area = width * height
    return area

def detect_trafficLight(img,x1,y1,x2,y2):
    
    crop_img = img[y1:y2,x1:x2]
    # Convert cropped image to HSV color space
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

    # Define lower and upper HSV color ranges for green, orange, and red
    green_lower = (40, 50, 50)
    green_upper = (80, 255, 255)
    yellow_lower = (20, 50, 50)
    yellow_upper = (40, 255, 255)
    red_lower = (0, 50, 50)
    red_upper = (10, 255, 255)

    # Create binary masks for each color
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    orange_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    red_mask = cv2.inRange(hsv, red_lower, red_upper)

    # Create range of coordinates that ensure only detect center traffic light
    if x1 > 200 and x2 < 600:
        in_range = True
    else:
        in_range = False

    # Check if any pixels are in each binary mask and print the color to the console
    if cv2.countNonZero(green_mask) > 0 and in_range:
        print("3") # Green
    elif cv2.countNonZero(orange_mask) > 0 and in_range:
        print("2") # Yellow
    elif cv2.countNonZero(red_mask) > 0 and in_range:
        print("1") # Red
    else:
        print("0") # No traffic light detected

# Assuming we should use ros com instead cuz we are not controlling the car here so we just need to read the code colour ryg or nothing and move accordingly.
# Problem comes with what rostopic is this under. then we just need to read it since the words can only be the 4 types gyr no traffic we can technically hard code it
# being that only yellow or red it will proceed to decelerate if not it will continue to move.

# Something like this except this is not using rosnodes.
# def traffic_stop(img,x1,x2,y1,y2):
     # if cv2.countNonZero(green_mask) > 0 and in_range:
        # move
     # else stop


# test
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

                # check traffic light state
                if int(index) == 9 and probability > 30:
                    detect_trafficLight(img,x1,y1,x2,y2)

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
    resize = cv2.resize(cv_image, (1280,1280))
    resize1 = resize[300:1000,300:1000]
    resized = cv2.resize(resize1,(1280,1280))
    output = model(resized)
    show_image(resized,output)
 

rospy.init_node('opencv_example',anonymous=True)
rospy.loginfo("Hello ROS!")

while not rospy.is_shutdown():
    # sub_image = rospy.Subscriber("/carla/ego_vehicle/rgb_top/image", Image, image_callback)
    sub_image = rospy.Subscriber("/carla/ego_vehicle/rgb_front/image", Image, image_callback)
    rospy.spin()

# Problems traffic light of other lanes is detected and stll cant detect own lane cuz too far