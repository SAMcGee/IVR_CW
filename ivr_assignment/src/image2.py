#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
import image1 as img1
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # Additional Code Added
        self.t0 = rospy.get_time()

        self.joint1 = Float64()
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()

        # used to track what the previous estimation of this joint was
        self.joint2_angle_prev = 0
        self.joint3_angle_prev = 0
        self.joint4_angle_prev = 0

        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        self.robot_joint2_angle_estimate_pub = rospy.Publisher("/robot/joint2_angle_est", Float64, queue_size=10)
        self.robot_joint3_angle_estimate_pub = rospy.Publisher("/robot/joint3_angle_est", Float64, queue_size=10)
        self.robot_joint4_angle_estimate_pub = rospy.Publisher("/robot/joint4_angle_est", Float64, queue_size=10)

        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

    def callback1(self, data):
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.waitKey(1)
        # cv2.imwrite('image_1.png', self.cv_image1)
        # j3 = cv2.imshow('window1', self.cv_image1)

    # Receive data, process it, and publish
    def callback2(self, data):
        # Receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # cv2.imwrite('image_2.png', self.cv_image2)
        # j2 = cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        self.move_robot()
        self.joints = Float64MultiArray
        self.joints.data = self.detect_joint_angles(self.cv_image1, self.cv_image2)

        # Publish the results
        try:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))

            self.robot_joint2_angle_estimate_pub.publish(self.joints.data[0])
            self.robot_joint3_angle_estimate_pub.publish(self.joints.data[1])
            self.robot_joint4_angle_estimate_pub.publish(self.joints.data[2])

        except CvBridgeError as e:
            print(e)

    def move_robot(self):
        cur_time = np.array([rospy.get_time()]) - self.t0

        self.joint1.data = 0  # Joint 1 is fixed
        self.joint2.data = (np.pi / 2) * np.sin(cur_time *np.pi / 15)
        self.joint3.data = (np.pi / 2) * np.sin(cur_time * np.pi / 18)
        self.joint4.data = (np.pi / 2) * np.sin(cur_time * np.pi / 20)

        self.robot_joint2_pub.publish(self.joint2)
        self.robot_joint3_pub.publish(self.joint3)
        self.robot_joint4_pub.publish(self.joint4)

    def detect_color(self, image, color):
        if color == "blue":
            mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        elif color == "green":
            mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        elif color == "red":
            mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        elif color == "yellow":
            mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        elif color == "orange":
            mask = cv2.inRange(image, (0, 100, 100), (0, 128, 255))
        else:
            raise Exception("Attempting to detect undefined color")

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        moments = cv2.moments(mask)

        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        return np.array([cx, cy])

    def pixel2meter(self, image):
        circle1Pos = self.detect_color(image, "yellow")
        circle2Pos = self.detect_color(image, "blue")
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        # 2.5 is the length of the first link (between yellow and blue)
        return 2.5 / np.sqrt(dist)

    def detect_joint_angles(self,image1 , image2):
        a = self.pixel2meter(image1) # Will be the same in second image because it doesn't move

        # Centre-points of each blob calculated using the first image
        im1center = a * self.detect_color(image1, "yellow")
        im1circle1Pos = a * self.detect_color(image1, "blue")
        im1circle2Pos = a * self.detect_color(image1, "green")
        im1circle3Pos = a * self.detect_color(image1, "red")

        # Centre-points of each blob calculated using the second image
        im2center = a * self.detect_color(image2, "yellow")
        im2circle1Pos = a * self.detect_color(image2, "blue")
        im2circle2Pos = a * self.detect_color(image2, "green")
        im2circle3Pos = a * self.detect_color(image2, "red")

        # Camera 1 gives us (y,z)
        # Camera 2 gives us (x,z)
        # So z should be the same across either, we derive the x,y,z positions as follows
        # There are small differences in z between them but it won't effect calculations

        yellowCoordinates = np.array([im2center[0],im1center[0],im2center[1]])
        blueCoordinates = np.array([im2circle1Pos[0], im1circle1Pos[0], im2circle1Pos[1]])
        greenCoordinates = np.array([im2circle2Pos[0], im1circle2Pos[0], im2circle2Pos[1]])
        redCoordinates = np.array([im2circle3Pos[0], im1circle3Pos[0], im2circle3Pos[1]])

        # Arctan2 takes dy, dx as arguments

        # ja1 doesn't rotate so is always 0 but could be calculated like this
        # ja1 = np.arctan2(yellowCoordinates[2] - blueCoordinates[2],
        #                  blueCoordinates[1] - yellowCoordinates[1])
        ja1 = 0

        # ja2 rotates in x
        print(blueCoordinates[2],greenCoordinates[2],blueCoordinates[1], greenCoordinates[1])
        ja2 = np.arctan2(blueCoordinates[2] - greenCoordinates[2],
                         blueCoordinates[1] - greenCoordinates[1]) - ja1

        # ja3 rotates in y
        ja3 = np.arctan2(greenCoordinates[1] - blueCoordinates[1],
                         greenCoordinates[0] - blueCoordinates[0]) - ja2 -ja1

        # ja4 rotates in x
        ja4 = np.arctan2(redCoordinates[2] - blueCoordinates[1],
                         redCoordinates[1] - blueCoordinates[1]) - ja3 - ja2 - ja1

        # ja2 = self.compensate_joint(ja2, self.joint2_angle_prev, 0.1, 0.5)
        # ja3 = self.compensate_joint(ja3, self.joint3_angle_prev, 0.1, 0.5)
        # ja4 = self.compensate_joint(ja3, self.joint3_angle_prev, 0.1, 0.5)

        self.joint2_angle_prev = ja2
        self.joint3_angle_prev = ja3
        self.joint3_angle_prev = ja4

        return np.array([ja2,ja3,ja4])

    def compensate_joint(self, joint_value, prev_value, increment, threshold):
        # If the joint angle is a threshold different from the previous angle
        # returns the value of that joint incremented in the appropiate direction of
        # movement
        return_value = joint_value
        if (abs(joint_value - prev_value) > threshold):
            if (joint_value > prev_value):
                return_value = prev_value + 0.1
            else:
                return_value = prev_value - 0.1
        return return_value

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
