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

        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()

        # used to track what the previous estimation of this joint angle
        self.joint2_angle_prev = [0,0]
        self.joint3_angle_prev = [0,0]
        self.joint4_angle_prev = [0,0]

        # Publishers for joint angles
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)

        # Publishers for estimated joint angles
        self.robot_joint2_angle_estimate_pub = rospy.Publisher("/robot/joint2_angle_est", Float64, queue_size=10)
        self.robot_joint3_angle_estimate_pub = rospy.Publisher("/robot/joint3_angle_est", Float64, queue_size=10)
        self.robot_joint4_angle_estimate_pub = rospy.Publisher("/robot/joint4_angle_est", Float64, queue_size=10)
        
        # Publishers for estimated end-effector position
        self.robot_red_blob_pub = rospy.Publisher("/robot/red_blob_position", Float64MultiArray, queue_size=10)

        # Subscribers to recieve both images
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)

    def callback1(self, data):
        # Recieve image1 (from camera one)
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.waitKey(1)

    def callback2(self, data):
        # Receive image2 (from camera two)
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.waitKey(1)

        self.move_robot()
        self.joints = Float64MultiArray
        self.red_blob_loc = Float64MultiArray
        self.joints.data = self.detect_joint_angles(self.cv_image1, self.cv_image2)

        # Publish the results
        try:
            self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
            
            self.robot_red_blob_pub.publish(self.red_blob_loc)
            
            self.robot_joint2_angle_estimate_pub.publish(self.joints.data[0])
            self.robot_joint3_angle_estimate_pub.publish(self.joints.data[1])
            self.robot_joint4_angle_estimate_pub.publish(self.joints.data[2])

            self.robot_joint2_pub.publish(self.joint2)
            self.robot_joint3_pub.publish(self.joint3)
            self.robot_joint4_pub.publish(self.joint4)

        except CvBridgeError as e:
            print(e)

    def move_robot(self):
        cur_time = np.array([rospy.get_time()]) - self.t0

        self.joint2.data = (np.pi / 2) * np.sin(cur_time * np.pi / 15)
        self.joint3.data = (np.pi / 2) * np.sin(cur_time * np.pi / 18)
        self.joint4.data = (np.pi / 2) * np.sin(cur_time * np.pi / 20)
        if cur_time > 50000:
            self.t0 = rospy.get_time()

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

        # In the case no blob can be found, returns a value outside the range
        # so the compensate_joint function will produce estimated values
        # until the joint is visible again
        if moments['m00'] == 0:
            return np.array([5,5])

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
        im1_yellow = a * self.detect_color(image1, "yellow")
        im1_blue = a * self.detect_color(image1, "blue")
        im1_green = a * self.detect_color(image1, "green")
        im1_red = a * self.detect_color(image1, "red")

        # Centre-points of each blob calculated using the second image
        im2_yellow = a * self.detect_color(image2, "yellow")
        im2_blue = a * self.detect_color(image2, "blue")
        im2_green = a * self.detect_color(image2, "green")
        im2_red = a * self.detect_color(image2, "red")
        
        self.red_blob_loc = np.array([im2_red[0], im1_red[0], im2_red[1]])

        # Camera 1 gives us (y,z)
        # Camera 2 gives us (x,z)
        # So z should be the same across either, we derive the x,y,z positions as follows
        # There are small differences in z between them but it won't effect calculations

        yellowCoordinates = np.array([im2_yellow[0],im1_yellow[0],im2_yellow[1]])
        blueCoordinates = np.array([im2_blue[0], im1_blue[0], im2_blue[1]])
        greenCoordinates = np.array([im2_green[0], im1_green[0], im2_green[1]])
        redCoordinates = np.array([im2_red[0], im1_red[0], im2_red[1]])

        # ja2 rotates in x (ja1 doesn't rotate)
        # The 0.2 is an error value as the joint is consistenly overstimated by 0.2 radians
        ja2 = np.arctan2(blueCoordinates[1] - greenCoordinates[1],
                         blueCoordinates[2] - greenCoordinates[2]) -0.2

        # ja3 rotates in y (no other joints rotate in y)
        ja3 = np.arctan2(greenCoordinates[0] - blueCoordinates[0],
                         blueCoordinates[2] - greenCoordinates[2])

        # ja4 rotates in x (hence we need only minus ja2 from it)
        ja4 = np.arctan2(greenCoordinates[1] - redCoordinates[1],
                         greenCoordinates[2] - redCoordinates[2]) - ja2

        ja2 = self.compensate_joint(ja2, self.joint2_angle_prev, 0.1, 1)
        ja3 = self.compensate_joint(ja3, self.joint3_angle_prev, 0.1, 1)
        ja4 = self.compensate_joint(ja4, self.joint4_angle_prev, 0.1, 1)
        self.joint2_angle_prev = [self.joint2_angle_prev[1], ja2]
        self.joint3_angle_prev = [self.joint3_angle_prev[1], ja3]
        self.joint4_angle_prev = [self.joint4_angle_prev[1], ja4]

        return np.array([ja2,ja3,ja4])

    def compensate_joint(self, joint_value, prev_value, increment, threshold):
        # If the joint angle is a threshold different from the previous angle
        # returns the value of that joint incremented in the appropiate direction of
        # movement. pi/2 and -pi/2 are the maximum and minimum values of the joints
        return_value = joint_value

        if (joint_value > np.pi/1.2):
            return prev_value[1] - increment

        if (joint_value < -1 * np.pi/1.2):
            return prev_value[1] + increment

        if (abs(joint_value - prev_value[1]) > threshold):

            if prev_value[0] < prev_value[1]:

                if prev_value[1] + (prev_value[1] - prev_value[0]) < np.pi/2:
                    return_value = prev_value[1] + (prev_value[1] - prev_value[0])
                else:
                    return_value = prev_value[1] - (prev_value[1] - prev_value[0])

            elif prev_value[0] > prev_value[1]:

                if prev_value[1] - (prev_value[0] - prev_value[1]) > -1 * np.pi/2:
                    return_value = prev_value[1] - (prev_value[0] - prev_value[1])
                else:
                    return_value = prev_value[1] + (prev_value[0] - prev_value[1])

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
