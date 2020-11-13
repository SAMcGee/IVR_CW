#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send images from camera2 to a topic named image_topic2
        self.image_pub2 = rospy.Publisher("image_topic2", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        # Additional Code Added
        self.t0 = rospy.get_time()

        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

    # Receive data, process it, and publish
    def callback2(self, data):
        # Receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Uncomment if you want to save the image
        # cv2.imwrite('image_copy.png', cv_image)
        im2 = cv2.imshow('window2', self.cv_image2)
        cv2.waitKey(1)

        cur_time = np.array([rospy.get_time()]) - self.t0
        joint1 = Float64()
        joint1.data = 2.5 * np.cos(cur_time * np.pi / 15)
        joint2 = Float64()
        joint2.data = 2.5 * np.sin(cur_time * np.pi / 15)
        joint3 = Float64()
        joint3.data = 1 * np.sin(cur_time * np.pi / 15)
        self.robot_joint2_pub.publish(joint1)
        self.robot_joint3_pub.publish(joint2)
        self.robot_joint4_pub.publish(joint3)

        # Publish the results
        try:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))

        except CvBridgeError as e:
            print(e)

    def move_robot(self):
        cur_time = np.array([rospy.get_time()]) - self.t0
        # Joint 1 is fixed

        joint2 = Float64()
        joint2.data = (np.pi / 2) * np.sin(cur_time * np.pi / 15)
        joint3 = Float64()
        joint3.data = (np.pi / 2) * np.sin(cur_time * np.pi / 18)
        joint4 = Float64()
        joint4.data = (np.pi / 2) * np.sin(cur_time * np.pi / 20)
        self.robot_joint2_pub.publish(joint2)
        self.robot_joint3_pub.publish(joint3)
        self.robot_joint4_pub.publish(joint4)

    '''
    Returns midpoint of the circle of the specified color in the image provided.
    '''

    def detect_color(image, color):
        if color == "blue":
            mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
        elif color == "green":
            mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
        elif color == "red":
            mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
        elif color == "yellow":
            mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
        else:
            raise Exception("Attempting to detect undefined color")

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=3)
        moments = cv2.moments(mask)
        cx = int(moments['m10'] / moments('m00'))
        cy = int(moments['m01'] / moments('m00'))
        return np.array([cx, cy])

    def pixel2meter(self, image):
        circle1Pos = self.detect_color(image, "yellow")
        circle2Pos = self.detect_color(image, "blue")
        dist = np.sum((circle1Pos - circle2Pos) ** 2)
        # 2.5 is the length of the first link (between yellow and blue)
        return (2.5 / np.sqrt(dist))

    '''
    Currently erroneously assuming that all joints are always visible in one camera
    The blue joint however rotates around both y and x axes, so this is not the case
    and both cameras will need to be used to accomodate for this 
    '''
    def detect_joint_angles(self,image):
        a = self.pixel2meter(self,image)
        # Centre-points of each blob
        center = a * self.detect_color("yellow")
        circle1Pos = a * self.detect_color("blue")
        circle2Pos = a * self.detect_color("green")
        circle3Pos = a * self.detect_color("red")
        # Joint angles, solved with trig
        ja1 = np.arctan2(center[0] - circle1Pos[0], center[1] - circle1Pos[1])
        ja2 = np.arctan2(circle1Pos[0] - circle2Pos[0], circle1Pos[1] - circle2Pos[1]) - ja1
        ja3 = np.arctan2(circle2Pos[0] - circle3Pos[0], circle2Pos[1] - circle3Pos[1]) - ja2 - ja1

        return np.array([ja1,ja2,ja3])

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
