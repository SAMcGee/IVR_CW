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
    
    self.previous_joint1 = Float64()
    self.previous_joint2 = Float64()
    self.previous_joint3 = Float64()
    self.previous_joint4 = Float64()
    
    self.time_previous_step = np.array([rospy.get_time()],dtype='float64')
    
    self.error = np.array([0.0,0.0,0.0], dtype='float64')
    self.error_d = np.array([0.0,0.0,0.0], dtype='float64')
    
    # Subscriber for actual joint angles given to rostopic
    self.joint_sub = rospy.Subscriber("/robot/joint_states", JointState, self.joint_callback)   
    
    self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
    self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10) 
    self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10) 
    self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
    
    self.fk_x_pub = rospy.Publisher('fk_x', Float64, queue_size=10)
    self.fk_y_pub = rospy.Publisher('fk_y', Float64, queue_size=10)
    self.fk_z_pub = rospy.Publisher('fk_z', Float64, queue_size=10)
    
    self.target_x_sub = rospy.Subscriber('/target_x', Float64, self.target_x_callback)
    self.target_y_sub = rospy.Subscriber('/target_y', Float64, self.target_y_callback)
    self.target_z_sub = rospy.Subscriber('/target_z', Float64, self.target_z_callback)
    
    self.rectangle_x_sub = rospy.Subscriber('/target_rect_x', Float64, self.rectangle_x_callback)
    self.rectangle_y_sub = rospy.Subscriber('/target_rect_y', Float64, self.rectangle_y_callback)
    self.rectangle_z_sub = rospy.Subscriber('/target_rect_z', Float64, self.rectangle_z_callback)    
    
    self.end_effector_estimation = Float64MultiArray()
    self.end_effector_estimation.data = [0.0,0.0,0.0]
    self.target_x = Float64()    
    self.target_y = Float64()    
    self.target_z = Float64()
    self.rectangle_x = Float64()
    self.rectangle_y = Float64()
    self.rectangle_z = Float64()
    


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    ##im1=cv2.imshow('window1', self.cv_image1)
    ##im2=cv2.imshow('window2',self.cv_image2)
    cv2.waitKey(1)
    print('FK vs Image')
    fk_end_effector = self.forward_kinematics()
    print(fk_end_effector)
    print(self.actual_joint1)
    print(self.actual_joint2)
    print(self.actual_joint3)
    print(self.actual_joint4)
    
    self.fk_x = Float64()
    self.fk_x.data = fk_end_effector[0]
    self.fk_y = Float64()
    self.fk_y.data = fk_end_effector[1]
    self.fk_z = Float64()
    self.fk_z.data = fk_end_effector[2]
    
    
    image_end_effector = self.calculate_end_effector_coords(self.cv_image1,self.cv_image2)
    print(image_end_effector)
    
    
    
    #q_d = self.control_closed(self.cv_image1, self.cv_image2)
    #q_d = self.null_space()
    #q_d = [0.0,1.5,1.0,0.0]
    self.joint1=Float64()
    self.joint1.data = q_d[0]
    self.joint2=Float64()
    self.joint2.data = q_d[1]
    self.joint3=Float64()
    self.joint3.data = q_d[2]
    self.joint4=Float64()
    self.joint4.data = q_d[3]
    
    self.end_effector_estimation.data = [fk_end_effector[0],fk_end_effector[1],fk_end_effector[2]]

    
    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.robot_joint1_pub.publish(self.joint1)
      self.robot_joint2_pub.publish(self.joint2)
      self.robot_joint3_pub.publish(self.joint3)
      self.robot_joint4_pub.publish(self.joint4)
      self.fk_x_pub.publish(self.fk_x)
      self.fk_y_pub.publish(self.fk_y)
      self.fk_z_pub.publish(self.fk_z)
      #self.target_x_pub.publish(self.target_x)
      #self.target_y_pub.publish(self.target_y)
      #self.target_z_pub.publish(self.target_z)
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
    self.previous_joint1.data = self.actual_joint1.data
    self.previous_joint2.data = self.actual_joint2.data
    self.previous_joint3.data = self.actual_joint3.data
    self.previous_joint4.data = self.actual_joint4.data
    self.actual_joint1.data = data.position[0]
    self.actual_joint2.data = data.position[1]
    self.actual_joint3.data = data.position[2]
    self.actual_joint4.data = data.position[3]
    
  def target_x_callback(self, data):
    self.target_x = data  
    
  def target_y_callback(self, data):
    self.target_y = data  
    
  def target_z_callback(self, data):
    self.target_z = data
  
  def rectangle_x_callback(self, data):
    self.rectangle_x = data
  def rectangle_y_callback(self, data):
    self.rectangle_y = data
  def rectangle_z_callback(self, data):
    self.rectangle_z = data        
    
   
  
  def forward_kinematics(self):
    theta_1 = np.pi/2+self.actual_joint1.data
    theta_2 = -np.pi/2-self.actual_joint2.data
    theta_3 = self.actual_joint3.data
    theta_4 = self.actual_joint4.data
    
    theta_1 = np.pi/2+self.actual_joint1.data
    theta_2 = np.pi/2+self.actual_joint2.data
    theta_3 = self.actual_joint3.data
    theta_4 = self.actual_joint4.data
    
    end_effector = np.array([3.0*(sin(theta_1)*sin(theta_3) + cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_3) + 3.0*sin(theta_2)*sin(theta_4)*cos(theta_1) + 3.5*cos(theta_1)*cos(theta_2)*cos(theta_3), 3.0*(sin(theta_1)*cos(theta_2)*cos(theta_3) - sin(theta_3)*cos(theta_1))*cos(theta_4) + 3.0*sin(theta_1)*sin(theta_2)*sin(theta_4) + 3.5*sin(theta_1)*cos(theta_2)*cos(theta_3) - 3.5*sin(theta_3)*cos(theta_1), -3.0*sin(theta_2)*cos(theta_3)*cos(theta_4) - 3.5*sin(theta_2)*cos(theta_3) + 3.0*sin(theta_4)*cos(theta_2) + 2.5])
    
    end_effector = np.array([3.0*(sin(theta_1)*sin(theta_3) + cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_3) - 3.0*sin(theta_2)*sin(theta_4)*cos(theta_1) + 3.5*cos(theta_1)*cos(theta_2)*cos(theta_3),3.0*(sin(theta_1)*cos(theta_2)*cos(theta_3) - sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.0*sin(theta_1)*sin(theta_2)*sin(theta_4) + 3.5*sin(theta_1)*cos(theta_2)*cos(theta_3) - 3.5*sin(theta_3)*cos(theta_1),3.0*sin(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*sin(theta_2)*cos(theta_3) + 3.0*sin(theta_4)*cos(theta_2) + 2.5])
    
    end_effector = np.array([3.0*(sin(theta_1)*sin(theta_3) + cos(theta_1)*cos(theta_2)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_3) - 3.0*sin(theta_2)*sin(theta_4)*cos(theta_1) + 3.5*cos(theta_1)*cos(theta_2)*cos(theta_3),3.0*(sin(theta_1)*cos(theta_2)*cos(theta_3) - sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.0*sin(theta_1)*sin(theta_2)*sin(theta_4) + 3.5*sin(theta_1)*cos(theta_2)*cos(theta_3) - 3.5*sin(theta_3)*cos(theta_1),3.0*sin(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*sin(theta_2)*cos(theta_3) + 3.0*sin(theta_4)*cos(theta_2) + 2.5])
    return end_effector
    
  def calculate_jacobian(self):
    theta_1 = np.pi/2+self.actual_joint1.data
    theta_2 = -np.pi/2-self.actual_joint2.data
    theta_3 = self.actual_joint3.data
    theta_4 = self.actual_joint4.data
    
    theta_1 = np.pi/2+self.actual_joint1.data
    theta_2 = np.pi/2+self.actual_joint2.data
    theta_3 = self.actual_joint3.data
    theta_4 = self.actual_joint4.data
    theta_1 = np.pi/2
    
    jacobian = np.array([[3*(sin(theta_1)*sin(theta_3) - sin(theta_2)*cos(theta_1)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_3) - 3.5*sin(theta_2)*cos(theta_1)*cos(theta_3) + 3*sin(theta_4)*cos(theta_1)*cos(theta_2), -(3.0*sin(theta_2)*sin(theta_4) + 3.0*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*sin(theta_1), 3*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_2)*sin(theta_3) - 3.5*cos(theta_1)*cos(theta_3), 3*(sin(theta_1)*sin(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) + 3*sin(theta_1)*cos(theta_2)*cos(theta_4)],[-3*(sin(theta_1)*sin(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.5*sin(theta_1)*sin(theta_2)*cos(theta_3) + 3*sin(theta_1)*sin(theta_4)*cos(theta_2) - 3.5*sin(theta_3)*cos(theta_1), (3*sin(theta_2)*sin(theta_4) + 3*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*cos(theta_1), -3*(sin(theta_1)*cos(theta_3) + sin(theta_2)*sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.5*sin(theta_1)*cos(theta_3) - 3.5*sin(theta_2)*sin(theta_3)*cos(theta_1), 3*(sin(theta_1)*sin(theta_3) - sin(theta_2)*cos(theta_1)*cos(theta_3))*sin(theta_4) - 3*cos(theta_1)*cos(theta_2)*cos(theta_4)],[0, -3*sin(theta_2)*cos(theta_3)*cos(theta_4) - 3.5*sin(theta_2)*cos(theta_3) + 3*sin(theta_4)*cos(theta_2), -(3.0*cos(theta_4) + 3.5)*sin(theta_3)*cos(theta_2), 3*sin(theta_2)*cos(theta_4) - 3*sin(theta_4)*cos(theta_2)*cos(theta_3)]])
    
    jacobian = np.array([[3*(sin(theta_1)*sin(theta_3) - sin(theta_2)*cos(theta_1)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_3) - 3.5*sin(theta_2)*cos(theta_1)*cos(theta_3) + 3*sin(theta_4)*cos(theta_1)*cos(theta_2), -(3.0*sin(theta_2)*sin(theta_4) + 3.0*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*sin(theta_1), 3*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_2)*sin(theta_3) - 3.5*cos(theta_1)*cos(theta_3), 3*(sin(theta_1)*sin(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) + 3*sin(theta_1)*cos(theta_2)*cos(theta_4)],[-3*(sin(theta_1)*sin(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.5*sin(theta_1)*sin(theta_2)*cos(theta_3) + 3*sin(theta_1)*sin(theta_4)*cos(theta_2) - 3.5*sin(theta_3)*cos(theta_1), (3*sin(theta_2)*sin(theta_4) + 3*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*cos(theta_1), -3*(sin(theta_1)*cos(theta_3) + sin(theta_2)*sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.5*sin(theta_1)*cos(theta_3) - 3.5*sin(theta_2)*sin(theta_3)*cos(theta_1), 3*(sin(theta_1)*sin(theta_3) - sin(theta_2)*cos(theta_1)*cos(theta_3))*sin(theta_4) - 3*cos(theta_1)*cos(theta_2)*cos(theta_4)], [0, -3*sin(theta_2)*cos(theta_3)*cos(theta_4) - 3.5*sin(theta_2)*cos(theta_3) + 3*sin(theta_4)*cos(theta_2), -(3.0*cos(theta_4) + 3.5)*sin(theta_3)*cos(theta_2), 3*sin(theta_2)*cos(theta_4) - 3*sin(theta_4)*cos(theta_2)*cos(theta_3)]])
    
    jacobian = np.array([[3*(sin(theta_1)*sin(theta_3) - sin(theta_2)*cos(theta_1)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_3) - 3.5*sin(theta_2)*cos(theta_1)*cos(theta_3) + 3*sin(theta_4)*cos(theta_1)*cos(theta_2), -(3.0*sin(theta_2)*sin(theta_4) + 3.0*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*sin(theta_1), 3*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_2)*sin(theta_3) - 3.5*cos(theta_1)*cos(theta_3), 3*(sin(theta_1)*sin(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) + 3*sin(theta_1)*cos(theta_2)*cos(theta_4)],[-3*(sin(theta_1)*sin(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.5*sin(theta_1)*sin(theta_2)*cos(theta_3) + 3*sin(theta_1)*sin(theta_4)*cos(theta_2) - 3.5*sin(theta_3)*cos(theta_1), (3*sin(theta_2)*sin(theta_4) + 3*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*cos(theta_1), -3*(sin(theta_1)*cos(theta_3) + sin(theta_2)*sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.5*sin(theta_1)*cos(theta_3) - 3.5*sin(theta_2)*sin(theta_3)*cos(theta_1), 3*(sin(theta_1)*sin(theta_3) - sin(theta_2)*cos(theta_1)*cos(theta_3))*sin(theta_4) - 3*cos(theta_1)*cos(theta_2)*cos(theta_4)],[0, -3*sin(theta_2)*cos(theta_3)*cos(theta_4) - 3.5*sin(theta_2)*cos(theta_3) + 3*sin(theta_4)*cos(theta_2), -(3.0*cos(theta_4) + 3.5)*sin(theta_3)*cos(theta_2), 3*sin(theta_2)*cos(theta_4) - 3*sin(theta_4)*cos(theta_2)*cos(theta_3)]])
    
    jacobian = np.array([[0, -(3.0*sin(theta_2)*sin(theta_4) + 3.0*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*sin(theta_1), 3*(sin(theta_1)*sin(theta_2)*sin(theta_3) - cos(theta_1)*cos(theta_3))*cos(theta_4) + 3.5*sin(theta_1)*sin(theta_2)*sin(theta_3) - 3.5*cos(theta_1)*cos(theta_3), 3*(sin(theta_1)*sin(theta_2)*cos(theta_3) + sin(theta_3)*cos(theta_1))*sin(theta_4) + 3*sin(theta_1)*cos(theta_2)*cos(theta_4)],[0, (3*sin(theta_2)*sin(theta_4) + 3*cos(theta_2)*cos(theta_3)*cos(theta_4) + 3.5*cos(theta_2)*cos(theta_3))*cos(theta_1), -3*(sin(theta_1)*cos(theta_3) + sin(theta_2)*sin(theta_3)*cos(theta_1))*cos(theta_4) - 3.5*sin(theta_1)*cos(theta_3) - 3.5*sin(theta_2)*sin(theta_3)*cos(theta_1), 3*(sin(theta_1)*sin(theta_3) - sin(theta_2)*cos(theta_1)*cos(theta_3))*sin(theta_4) - 3*cos(theta_1)*cos(theta_2)*cos(theta_4)],[0, -3*sin(theta_2)*cos(theta_3)*cos(theta_4) - 3.5*sin(theta_2)*cos(theta_3) + 3*sin(theta_4)*cos(theta_2), -(3.0*cos(theta_4) + 3.5)*sin(theta_3)*cos(theta_2), 3*sin(theta_2)*cos(theta_4) - 3*sin(theta_4)*cos(theta_2)*cos(theta_3)]])

    return jacobian


    
  def control_closed(self,image1,image2):
    kp = 0.5
    kd = 0.3
    K_p = np.array([[kp,0,0],[0,kp,0],[0,0,kp]])    
    K_d = np.array([[kd,0,0],[0,kd,0],[0,0,kd]])
    
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    
    pos = self.forward_kinematics()
    
    pos_d = np.array([self.target_x.data, self.target_y.data, self.target_z.data])
    self.error_d = ((pos_d - pos) - self.error)/dt
    
    self.error = pos_d-pos
    
    q = np.array([self.actual_joint1.data, self.actual_joint2.data, self.actual_joint3.data, self.actual_joint4.data])
    
    J_inv = np.linalg.pinv(self.calculate_jacobian())
    dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose())))
    q_d = q + (dt * dq_d)
    return q_d
    
  def null_space(self):
    kp = 0.45
    kd = 0.1
    K_p = np.array([[kp,0,0],[0,kp,0],[0,0,kp]])
    
    K_d = np.array([[kd,0,0],[0,kd,0],[0,0,kd]])
    
    cur_time = np.array([rospy.get_time()])
    dt = cur_time - self.time_previous_step
    self.time_previous_step = cur_time
    rectangle_pos = np.array([self.rectangle_x.data, self.rectangle_y.data, self.rectangle_z.data])
    prev_pos = np.array([self.end_effector_estimation.data[0], self.end_effector_estimation.data[1], self.end_effector_estimation.data[2]])
    pos = self.forward_kinematics()
    
    pos_d = np.array([self.target_x.data, self.target_y.data, self.target_z.data])
    
    self.error_d = ((pos_d - pos) - self.error)/dt
    
    self.error = pos_d-pos

    
    q = np.array([self.actual_joint1.data, self.actual_joint2.data, self.actual_joint3.data, self.actual_joint4.data])
    q_prev = np.array([self.previous_joint1.data, self.previous_joint2.data, self.previous_joint3.data, self.previous_joint4.data])
    q_diff = q - q_prev
    wprev = np.linalg.norm(prev_pos - rectangle_pos)
    w = np.linalg.norm(pos - rectangle_pos)
    w_diff = w - wprev
    
    dq0 = np.array([(w_diff/q_diff[0]), w_diff/q_diff[1], w_diff/q_diff[2], w_diff/q_diff[3]])
    J = self.calculate_jacobian()
    J_inv = np.linalg.pinv(J)
    dq_d = np.dot(J_inv, (np.dot(K_d, self.error_d.transpose()) + np.dot(K_p, self.error.transpose()))) + np.dot((np.eye(4,4) - np.dot(J_inv,J)),dq0)
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


