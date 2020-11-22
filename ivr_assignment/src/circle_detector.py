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
        # initialize a publisher to send images from camera1 to a topic named image_topic1
        self.image_pub1 = rospy.Publisher("image_topic1", Image, queue_size=1)
        # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()

        self.target_pub_x = rospy.Publisher("/target_x", Float64, queue_size=1)
        self.target_pub_y = rospy.Publisher("/target_y", Float64, queue_size=1)
        self.target_pub_z = rospy.Publisher("/target_z", Float64, queue_size=1)

        self.chamfer_img = cv2.imread('template.png', 0)
        self.chamfer_img = self.chamfer_img.astype(np.uint8)

        self.t0 = rospy.get_time()

        self.prev_x = 0
        self.prev_y = 0
        self.prev_z = 0

        self.init_flag = True

    # Recieve data from camera 1, process it, and publish
    def callback1(self, data):
        # Recieve the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

    def callback2(self, data):
        # Receive image2 (from camera two)
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.waitKey(1)
        coords = self.get_circle_coords()

        self.target_pub_x.publish(coords[0])
        self.target_pub_y.publish(coords[1])
        self.target_pub_z.publish(coords[2])

    def get_circle_coords(self):

        # Camera 1 gives us (y,z) of the orange sphere
        # Camera 2 gives us (x,z) of the orange sphere
        im1center = self.detect_orange(self.cv_image1)
        im2center = self.detect_orange(self.cv_image2)

        if not self.init_flag:
            return_x = self.adjust_for_going_oob(im2center[0], 50, 'x')
            return_y = self.adjust_for_going_oob(im1center[0], 30, 'y')
            return_z = self.adjust_for_going_oob(im2center[1], 50, 'z')
        else:
            return_x = int(im2center[0])
            return_y = int(im1center[0])
            return_z = int(im2center[1])
            self.init_flag = False

        self.prev_x = return_x
        self.prev_y = return_y
        self.prev_z = return_z

        # cv2.circle(self.cv_image1, (return_y, return_z), 3, 255, 2)
        # cv2.imshow('cam1', self.cv_image1)
        #
        # cv2.circle(self.cv_image2, (int(return_x), int(return_z)), 3, 255, 2)
        # cv2.imshow('cam2', self.cv_image2)

        return(np.array([return_x, return_y, return_z]))

    # Adjusts for the sphere going out of bounds by comparing to previous location
    def adjust_for_going_oob(self, cur, threshold, dimension):
        if dimension == 'x':
            prev = self.prev_x
        elif dimension == 'y':
            prev = self.prev_y
        elif dimension == 'z':
            prev = self.prev_z
        else:
            Exception('Dimension' + str(dimension) + ' is invalid')

        if abs(cur - prev) > threshold:
            return prev
        else:
            return cur

    # Detect and show orange
    def detect_orange(self, image):
        # Mask is used to show everything which is orange in the scene
        mask = cv2.inRange(image, (1, 30, 100), (50, 170, 255))
        kernel = np.ones((5, 5), np.uint8)

        moments = cv2.moments(mask)
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])
        w, h = self.chamfer_img.shape[::-1]

        # We defined a region of interest (ROI) as 150 pixels around the center
        ROI = mask[int(cy - 150): int(cy + 150) + 1,
              int(cx - 150): int(cx + 150) + 1]

        # Matches a cropped template against the ROI
        results = cv2.matchTemplate(ROI, self.chamfer_img, cv2.TM_CCORR_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(results)

        # Adjusts the location within the ROI to be the location in the larger image
        img_x = int((cx - 150) + max_loc[0] + w/2)
        img_y = int((cy - 150) + max_loc[1] + h/2)

        # cv2.circle(self.cv_image1, (img_x, img_y), 3, 255, 2)
        # cv2.imshow('result', self.cv_image1)

        return np.array([img_x, img_y])

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