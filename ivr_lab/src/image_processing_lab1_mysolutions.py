#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

from matplotlib import pyplot as plt


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
                # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send messages to a topic named image_topic
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        # initialize a subscriber to receive messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub = rospy.Subscriber("/robot/camera1/image_raw", Image, self.callback)
        #(for lab3)
        # initialize a publisher to send robot end-effector position
        self.end_effector_pub = rospy.Publisher("end_effector_prediction", Float64MultiArray, queue_size=10)
        # initialize a publisher to send desired trajectory
        self.trajectory_pub = rospy.Publisher("trajectory", Float64MultiArray, queue_size=10)
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        # record the beginning time
        self.time_trajectory = rospy.get_time()

    # Define a circular trajectory (for lab 3)
    def trajectory(self):
        # get current time
        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        x_d = float(6 * np.cos(cur_time * np.pi / 100))
        y_d = float(6 + np.absolute(1.5 * np.sin(cur_time * np.pi / 100)))
        return np.array([x_d, y_d])

    # Receive data, process it, and publish
    def callback(self, data):
        # Receive the image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow('window', cv_image)
        cv2.waitKey(3)

        # change te value of self.joint.data to your estimated value from thew images once you have finalized the code
        self.joints = Float64MultiArray()
        self.joints.data = np.array([0, 0, 0])
        # loading template for links as binary image (used in lab 2)
        self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
        self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
        self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))

        # Perform image processing task (your code goes here)
        # The image is loaded as cv_image
        # (x1,y1) = detect_red(cv_image)
        # (x2,y2) = detect_green(cv_image)

        # print(get_angle("yellow",cv_image))
        # detect_blue(cv_image)
        # detect_yellow(cv_image)
        detect_l3(self,cv_image, 10)

        # Uncomment if you want to save the image
        cv2.imwrite('image_copy.png', cv_image)

        # publish the estimated position of robot end-effector (for lab 3)
        # x_e_image = np.array([0, 0, 0])
        # self.end_effector=Float64MultiArray()
        # self.end_effector.data= x_e_image

        # send control commands to joints (for lab 3)
        # self.joint1=Float64()
        # self.joint1.data= q_d[0]
        # self.joint2=Float64()
        # self.joint2.data= q_d[1]
        # self.joint3=Float64()
        # self.joint3.data= q_d[2]

        # Publishing the desired trajectory on a topic named trajectory(for lab 3)
        # x_d = self.trajectory()    # getting the desired trajectory
        # self.trajectory_desired= Float64MultiArray()
        # self.trajectory_desired.data=x_d

        # Publish the results - the images are published under a topic named "image_topic" and calculated joints angles are published under a topic named "joints_pos"
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.joints_pub.publish(self.joints)
            # (for lab 3)
            # self.trajectory_pub.publish(self.trajectory_desired)
            # self.end_effector_pub.publish(self.end_effector)
            # self.robot_joint1_pub.publish(self.joint1)
            # self.robot_joint2_pub.publish(self.joint2)
            # self.robot_joint3_pub.publish(self.joint3)
        except CvBridgeError as e:
            print(e)

# Remember that in openCV color code is GBR 
def detect_red(image):
    bgr = [5,5,180]
    red_thresh = 85
    othercols_thresh = 10
    
    minBGR = np.array([bgr[0] - othercols_thresh, bgr[1] - othercols_thresh, bgr[2] - red_thresh])
    maxBGR = np.array([bgr[0] + othercols_thresh, bgr[1] + othercols_thresh, bgr[2] + red_thresh])
    
    maskBGR = cv2.inRange(image, minBGR, maxBGR)
    resultBGR = cv2.bitwise_and(image,image,mask = maskBGR)
    # cv2.imshow("Result BGR", resultBGR)
    cv2.imwrite('red_image.png', resultBGR)
    
    gray_image = cv2.cvtColor(resultBGR, cv2.COLOR_BGR2GRAY)
    M = cv2.moments(gray_image)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # cv2.circle(resultBGR, (cX, cY), 1, (255,255,255),-1)
    # cv2.imwrite("red_centroid.png", resultBGR)
    return(cX,cY)
        
def detect_green(image):
    bgr = [5,180,5]
    green_thresh = 85
    othercols_thresh = 10
    
    minBGR = np.array([bgr[0] - othercols_thresh, bgr[1] - green_thresh, bgr[2] - othercols_thresh])
    maxBGR = np.array([bgr[0] + othercols_thresh, bgr[1] + green_thresh, bgr[2] + othercols_thresh])
    
    maskBGR = cv2.inRange(image, minBGR, maxBGR)
    resultBGR = cv2.bitwise_and(image,image,mask = maskBGR)
    # cv2.imshow("Result BGR", resultBGR)
    cv2.imwrite('green_image.png', resultBGR)
    
    gray_image = cv2.cvtColor(resultBGR, cv2.COLOR_BGR2GRAY)
    M = cv2.moments(gray_image)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # cv2.circle(resultBGR, (cX, cY), 1, (255,255,255),-1)
    # cv2.imwrite("green_centroid.png", resultBGR)
    return(cX,cY)

def detect_blue(image):
    bgr = [180,5,5]
    blue_thresh = 85
    othercols_thresh = 10
    
    minBGR = np.array([bgr[0] - blue_thresh, bgr[1] - othercols_thresh, bgr[2] - othercols_thresh])
    maxBGR = np.array([bgr[0] + blue_thresh, bgr[1] + othercols_thresh, bgr[2] + othercols_thresh])
    
    maskBGR = cv2.inRange(image, minBGR, maxBGR)
    resultBGR = cv2.bitwise_and(image,image,mask = maskBGR)
    # cv2.imshow("Result BGR", resultBGR)
    cv2.imwrite('blue_image.png', resultBGR)
    
    gray_image = cv2.cvtColor(resultBGR, cv2.COLOR_BGR2GRAY)
    M = cv2.moments(gray_image)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # cv2.circle(resultBGR, (cX, cY), 1, (255,255,255),-1)
    # cv2.imwrite("blue_centroid.png", resultBGR)
    return(cX,cY)
    
def detect_yellow(image):
    bgr = [0,180,180]
    redyellow_thresh = 85
    othercols_thresh = 0
    
    minBGR = np.array([bgr[0] - othercols_thresh, bgr[1] - redyellow_thresh, bgr[2] - redyellow_thresh])
    maxBGR = np.array([bgr[0] + othercols_thresh, bgr[1] + redyellow_thresh, bgr[2] + redyellow_thresh])
    
    maskBGR = cv2.inRange(image, minBGR, maxBGR)
    resultBGR = cv2.bitwise_and(image,image,mask = maskBGR)
    # cv2.imshow("Result BGR", resultBGR)
    cv2.imwrite('yellow_image.png', resultBGR)
    
    gray_image = cv2.cvtColor(resultBGR, cv2.COLOR_BGR2GRAY)
    M = cv2.moments(gray_image)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    # cv2.circle(resultBGR, (cX, cY), 1, (255,255,255),-1)
    # cv2.imwrite("yellow_centroid.png", resultBGR)
    return(cX,cY)

def get_angle(color, image):
    red_pos = detect_red(image)
    green_pos = detect_green(image)
    blue_pos = detect_blue(image)
    yellow_pos = detect_yellow(image)

    if (color == "red"):
        return(0)
    elif (color == "green"):
        return(math.atan2((red_pos[1] - green_pos[1]),(red_pos[0] - green_pos[0])))
    elif (color == "blue"):
        return(math.atan2((green_pos[1] - blue_pos[1]),(green_pos[0] - blue_pos[0])))
    elif (color == "yellow"):
        return(math.atan2((blue_pos[1] - yellow_pos[1]),(blue_pos[0] - yellow_pos[0])))
    else:
        print("invalid color provided")

# def match_link(image, template):
#     img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     img_gray = cv2.bitwise_not(img_gray)
#     w, h = template.shape[::-1]
#
#     res = cv2.matchTemplate(img_gray, template, cv2.TM_CCOEFF_NORMED)
#     threshold = 0.50
#     loc = np.where(res >= threshold)
#     loc = max(loc)
#
#     for pt in zip(*loc[::-1]):
#         cv2.rectangle(image, pt, (pt[0]+w, pt[1]+h), (0,255,255), 1)
#
#     cv2.imshow('detected_matches',image)
def detect_l3(self, image, quadrant):
    # find centre
    circle1Pos = detect_green(image)
    circle2Pos = detect_red(image)
    centre = (circle1Pos + circle2Pos) /2

    mask = cv2.inRange(image, (0,0,0), (1,1,1))
    ROI = mask[centre[1] - self.link3.shape[0] /2: (centre[1] + self.link3.shape[0]/2) + 1,
          centre[0] - self.link3.shape[1] / 2: (center[0] + self.link3.shape[1] /2) + 1
          ]
    ROI = ROI[O:self.link3.shape[0], 0:self.link3.shape[1]]

    dist = cv2.distanceTransform(cv2/bitwise_not(ROI), cv2.DIST_L2, 0)
    cv2.imshow('detected_matches', dist)

def euclidean_distance(x,y):
    running_total = 0
    for i in range(0,len(x)):
	    running_total += math.sqrt((x[i]-y[i]) ** 2)
    return(running_total)
    


# call the class
def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
