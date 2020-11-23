#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from math import sin, cos
from matplotlib import pyplot as plt

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    
    self.actual_joint1 = Float64()
    self.actual_joint2 = Float64()
    self.actual_joint3 = Float64()
    self.actual_joint4 = Float64()
    
    self.time_previous_step = np.array([rospy.get_time()],dtype='float64')
    
    self.error = np.array([0.0,0.0,0.0], dtype='float64')
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
    
    # Subscriber for actual joint angles given to rostopic
    self.joint_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_callback)   
    
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10) 
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10) 
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    self.figx, self.axx = plt.subplots()
    self.figy, self.axy = plt.subplots() 
    self.figz, self.axz = plt.subplots()  
     
    


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    im2=cv2.imshow('window2',self.cv_image2)
    cv2.waitKey(1)
    print('FK vs Image')
    fk_end_effector = self.forward_kinematics()
    print(fk_end_effector)
    image_end_effector = self.calculate_end_effector_coords(self.cv_image1,self.cv_image2)
    print(image_end_effector)
    target_position = self.calculate_target_position()
    self.plot_positions(fk_end_effector, target_position)
    
    q_d = self.control_closed(self.cv_image1, self.cv_image2)
    self.joint1=Float64()
    self.joint1.data = q_d[0]
    self.joint2=Float64()
    self.joint2.data = q_d[1]
    self.joint3=Float64()
    self.joint3.data = q_d[2]
    self.joint4=Float64()
    self.joint4.data = q_d[3]
    
    
    
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)
      
  def callback2(self, data):
        # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    cv2.waitKey(1)

  
  def joint_callback(self, data):
    self.actual_joint1.data = data.position[0]
    self.actual_joint2.data = data.position[1]
    self.actual_joint3.data = data.position[2]
    self.actual_joint4.data = data.position[3]
    
  def plot_positions(self, end_effector, target):
    time = rospy.get_time()
    figx, axx = self.figx,self.axx
    axx.plot(time,end_effector[0], '.', c='b')
    axx.plot(time,target[0],'.', c='orange')
    axx.set(xlim=([time - 10, time]))
    figx.canvas.draw()
    figx.savefig('plotx.png')
    
    figy, axy = self.figy,self.axy
    axy.plot(time,end_effector[1], '.', c='g')
    axy.plot(time,target[1],'.', c='orange')
    axy.set(xlim=([time - 10, time]))
    figy.canvas.draw()
    figy.savefig('ploty.png')
    
    figz, axz = self.figz, self.axz
    axz.plot(time,end_effector[2], '.', c='y')
    axz.plot(time,target[2],'.', c='orange')
    axz.set(xlim=([time - 10, time]))
    figz.canvas.draw()
    figz.savefig('plotz.png')
    plt.pause(0.0000000001)
  
  
  def forward_kinematics(self):
    theta_1 = -self.actual_joint1.data
    theta_2 = self.actual_joint2.data
    theta_3 = self.actual_joint3.data
    theta_4 = self.actual_joint4.data
    alpha_1 = 3*np.pi/2
    alpha_2 = 3*np.pi/2
    alpha_3 = np.pi/2
    pi = np.pi
    end_effector = np.array([3*((-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3) - (sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3))*cos(theta_4) + 3.5*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3) + 3*((-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3)*cos(alpha_3) + (sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*cos(alpha_3)*cos(theta_3) + (sin(alpha_1)*sin(3*pi/2 - theta_1)*cos(alpha_2) + sin(alpha_2)*sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(3*pi/2 + theta_2) + sin(alpha_2)*sin(3*pi/2 + theta_2)*cos(3*pi/2 - theta_1))*sin(alpha_3))*sin(theta_4) - 3.5*(sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3),3*((sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*cos(theta_3) - (-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) + cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3))*cos(theta_4) + 3.5*(sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*cos(theta_3) + 3*((sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*sin(theta_3)*cos(alpha_3) + (-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) + cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(alpha_3)*cos(theta_3) + (-sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1) + sin(alpha_2)*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2) - sin(alpha_2)*cos(alpha_1)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(alpha_3))*sin(theta_4) - 3.5*(-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) + cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3), 3*(-(sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(alpha_2)*cos(alpha_1))*sin(theta_3) + sin(alpha_1)*sin(3*pi/2 + theta_2)*cos(theta_3))*cos(theta_4) - 3.5*(sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(alpha_2)*cos(alpha_1))*sin(theta_3) + 3*((-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 + theta_2) + cos(alpha_1)*cos(alpha_2))*sin(alpha_3) + (sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(alpha_2)*cos(alpha_1))*cos(alpha_3)*cos(theta_3) + sin(alpha_1)*sin(theta_3)*sin(3*pi/2 + theta_2)*cos(alpha_3))*sin(theta_4) + 3.5*sin(alpha_1)*sin(3*pi/2 + theta_2)*cos(theta_3) + 2.5])
    return end_effector
    
  def calculate_jacobian(self):
    theta_1 = -self.actual_joint1.data
    theta_2 = self.actual_joint2.data
    theta_3 = self.actual_joint3.data
    theta_4 = self.actual_joint4.data
    alpha_1 = 3*np.pi/2
    alpha_2 = 3*np.pi/2
    alpha_3 = np.pi/2
    pi = np.pi
    
    jacobian = np.array([[(3*(sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*cos(theta_3) + 3*(sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) + sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) - cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3))*cos(theta_4) + (3.5*sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + 3.5*sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*cos(theta_3) + (3*(sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*sin(theta_3)*cos(alpha_3) + 3*(-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) + cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(alpha_3)*cos(theta_3) + 3*(-sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1) + sin(alpha_2)*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2) - sin(alpha_2)*cos(alpha_1)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(alpha_3))*sin(theta_4) + (3.5*sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) + 3.5*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) - 3.5*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3), (3*(-sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(3*pi/2 - theta_1))*cos(theta_3) + 3*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(alpha_2) + cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3))*cos(theta_4) + (-3.5*sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(3*pi/2 + theta_2) - 3.5*sin(3*pi/2 + theta_2)*cos(3*pi/2 - theta_1))*cos(theta_3) + (-3.5*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(alpha_2) + 3.5*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3) + (3*(-sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(3*pi/2 - theta_1))*sin(theta_3)*cos(alpha_3) + 3*(-sin(alpha_2)*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + sin(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(alpha_3) + 3*(sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(alpha_2) - cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(alpha_3)*cos(theta_3))*sin(theta_4), (-3*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3) + 3*(-sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) + sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*cos(theta_3))*cos(theta_4) + (3*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(alpha_3)*cos(theta_3) - 3*(sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3)*cos(alpha_3))*sin(theta_4) - (-3.5*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + 3.5*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3) + (-3.5*sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) + 3.5*sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + 3.5*sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*cos(theta_3), -(3*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3) - 3*(sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3))*sin(theta_4) + (3*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) + cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3)*cos(alpha_3) + 3*(sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*cos(alpha_3)*cos(theta_3) + 3*(sin(alpha_1)*sin(3*pi/2 - theta_1)*cos(alpha_2) + sin(alpha_2)*sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(3*pi/2 + theta_2) + sin(alpha_2)*sin(3*pi/2 + theta_2)*cos(3*pi/2 - theta_1))*sin(alpha_3))*cos(theta_4)]
, [(3*(sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) - cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3) + 3*(sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3))*cos(theta_4) + (3.5*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) - 3.5*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3) + (3*(sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_1) - cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3)*cos(alpha_3) + 3*(-sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) + sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*cos(alpha_3)*cos(theta_3) + 3*(-sin(alpha_1)*sin(3*pi/2 - theta_1)*cos(alpha_2) - sin(alpha_2)*sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(3*pi/2 + theta_2) - sin(alpha_2)*sin(3*pi/2 + theta_2)*cos(3*pi/2 - theta_1))*sin(alpha_3))*sin(theta_4) + (3.5*sin(alpha_1)*sin(alpha_2)*sin(3*pi/2 - theta_1) - 3.5*sin(3*pi/2 - theta_1)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - 3.5*sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3), (3*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2) + cos(alpha_1)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3) + 3*(sin(3*pi/2 - theta_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3))*cos(theta_4) + (-3.5*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2) + 3.5*cos(alpha_1)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3) + (3.5*sin(3*pi/2 - theta_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + 3.5*sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1))*sin(theta_3) + (3*(-sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2) + cos(alpha_1)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3)*cos(alpha_3) + 3*(sin(alpha_2)*sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(alpha_2)*sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*sin(alpha_3) + 3*(-sin(3*pi/2 - theta_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1))*cos(alpha_3)*cos(theta_3))*sin(theta_4), (-3*(sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*sin(theta_3) + 3*(sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) + sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) - cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3))*cos(theta_4) - (3.5*sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + 3.5*sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*sin(theta_3) + (3*(sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*cos(alpha_3)*cos(theta_3) - 3*(-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) + cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3)*cos(alpha_3))*sin(theta_4) + (3.5*sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) + 3.5*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) - 3.5*cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(theta_3), -(3*(sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*cos(theta_3) - 3*(-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) + cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(theta_3))*sin(theta_4) + (3*(sin(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2) + sin(3*pi/2 + theta_2)*cos(alpha_1)*cos(3*pi/2 - theta_1))*sin(theta_3)*cos(alpha_3) + 3*(-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 - theta_1) - sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2)*cos(alpha_2) + cos(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*cos(alpha_3)*cos(theta_3) + 3*(-sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 - theta_1) + sin(alpha_2)*sin(3*pi/2 - theta_1)*sin(3*pi/2 + theta_2) - sin(alpha_2)*cos(alpha_1)*cos(3*pi/2 - theta_1)*cos(3*pi/2 + theta_2))*sin(alpha_3))*cos(theta_4)]
, [0, (3*sin(alpha_1)*sin(theta_3)*sin(3*pi/2 + theta_2)*cos(alpha_2) + 3*sin(alpha_1)*cos(theta_3)*cos(3*pi/2 + theta_2))*cos(theta_4) + (3*sin(alpha_1)*sin(alpha_2)*sin(alpha_3)*sin(3*pi/2 + theta_2) + 3*sin(alpha_1)*sin(theta_3)*cos(alpha_3)*cos(3*pi/2 + theta_2) - 3*sin(alpha_1)*sin(3*pi/2 + theta_2)*cos(alpha_2)*cos(alpha_3)*cos(theta_3))*sin(theta_4) + 3.5*sin(alpha_1)*sin(theta_3)*sin(3*pi/2 + theta_2)*cos(alpha_2) + 3.5*sin(alpha_1)*cos(theta_3)*cos(3*pi/2 + theta_2), (3*(-sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - sin(alpha_2)*cos(alpha_1))*cos(theta_3) - 3*sin(alpha_1)*sin(theta_3)*sin(3*pi/2 + theta_2))*cos(theta_4) + (-3*(sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(alpha_2)*cos(alpha_1))*sin(theta_3)*cos(alpha_3) + 3*sin(alpha_1)*sin(3*pi/2 + theta_2)*cos(alpha_3)*cos(theta_3))*sin(theta_4) + (-3.5*sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) - 3.5*sin(alpha_2)*cos(alpha_1))*cos(theta_3) - 3.5*sin(alpha_1)*sin(theta_3)*sin(3*pi/2 + theta_2), -(-3*(sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(alpha_2)*cos(alpha_1))*sin(theta_3) + 3*sin(alpha_1)*sin(3*pi/2 + theta_2)*cos(theta_3))*sin(theta_4) + (3*(-sin(alpha_1)*sin(alpha_2)*cos(3*pi/2 + theta_2) + cos(alpha_1)*cos(alpha_2))*sin(alpha_3) + 3*(sin(alpha_1)*cos(alpha_2)*cos(3*pi/2 + theta_2) + sin(alpha_2)*cos(alpha_1))*cos(alpha_3)*cos(theta_3) + 3*sin(alpha_1)*sin(theta_3)*sin(3*pi/2 + theta_2)*cos(alpha_3))*cos(theta_4)]
])
    return jacobian


  def calculate_target_position(self):
    return np.array([4,2,5])
    
  def control_closed(self,image1,image2):
    K_p = np.array([[10,0,0],[0,10,0],[0,0,10]])
    
    K_d = np.array([[0.1,0,0],[0,0.1,0],[0,0,0.1]])
    
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    
    pos = self.forward_kinematics()
    
    pos_d = self.calculate_target_position()
    
    self.error_d = ((pos_d - pos) - self.error)/dt
    
    self.error = pos_d-pos
    
    q = np.array([self.actual_joint1.data, self.actual_joint2.data, self.actual_joint3.data, self.actual_joint4.data])
    
    J_inv = np.linalg.pinv(self.calculate_jacobian())
    dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))
    print(q.dtype)
    print(dt.dtype)
    print(dq_d.dtype)
    q_d = q + (dt * dq_d)
    return q_d
    
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
    
  def calculate_end_effector_coords(self,image1, image2):
    a = self.pixel2meter(image1)
    im1center = a * self.detect_color(image1, "yellow")
    im2center = a * self.detect_color(image2, "yellow")
    
    im1circle3Pos = a * self.detect_color(image1, "red")    
    im2circle3Pos = a * self.detect_color(image2, "red")
    
    yellowCoordinates = np.array([im2center[0],im1center[0],im2center[1]])
    
    redCoordinates = np.array([im2circle3Pos[0] - im2center[0], im1circle3Pos[0] - im1center[0], im2center[1] - im2circle3Pos[1]])
    return redCoordinates
    
    
  
    



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


