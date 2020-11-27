# IVR_CW
Introduction to Vision and Robotics
## How to Run the Programs

### Part 2

#### 2.1
  From anywhere within the catkin_ws folder run: 
  \
  ```
    rosrun ivr_assignment image2.py
  ```
  \
#### 2.2
  From `catkin_ws/src/IVR_CW/ivr_assignment/src` run the command:
  \
    ```
      rosrun ivr_assignment circle_detector.py
    ```
    \
### Part 3

#### 3.1 
  From anywhere within the catkin folder run: 
  \
    ```
      rosrun ivr_assignment image1.py
    ```
\
#### 3.2 
  First, in image1.py uncomment the 2 lines indicated below situated in `callback1(self,data)`:
  ``` 
    #CLOSED CONTROL - uncomment two lines below for closed control
    #q_d = self.control_closed()
    #q_d[0] = 0.0
  ```
  \
  Secondly, from `catkin_ws/src/IVR_CW/ivr_assignment/src` run the command:
  \
    ```
      rosrun ivr_assignment circle_detector.py
    ```
  \
  \
  And now from another terminal run:
  \
      ```
      rosrun ivr_assignment image1.py
      ```
### Part 4
#### 4.2 
  First, in image1.py uncomment the 2 lines indicated below situated in `callback1(self,data)`:
  ``` 
    #NULL-SPACE CONTROL - uncomment two lines below for null-space control
    #q_d = self.null_space()
    #q_d[0] = 0.0
  ```
  \
  Secondly, from `catkin_ws/src/IVR_CW/ivr_assignment/src` run the command:
  \
    ```
      rosrun ivr_assignment circle_detector.py
    ```
  \
  \
  Thirdly, from `catkin_ws/src/IVR_CW/ivr_assignment/src` run the command:
  \
    ```
      rosrun ivr_assignment rectangle_detector.py
    ```
  \
  \
  And now from another terminal run:
  \
      ```
      rosrun ivr_assignment image1.py
      ```
