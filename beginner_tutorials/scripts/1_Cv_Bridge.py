#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('beginner_tutorials')
import sys
import rospy
import cv2
import numpy as np
import operator
import scipy.io as spio
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
#from beginner_tutorials.msg import Foo
#centers = []
flag = 0
flag_2 = 0
state = 0
Color_Array = [1,1,1,1,1,1]
Board_2 = [0,0,0,0,0,0]
action_array = [0,0,0,0,0,0]
mat = spio.loadmat('/media/adarshjs/Productivity/Baxter Training/Baxter_Training_2.mat', squeeze_me=True)
Q_Table = mat['Q_Table']

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("Image_Publisher",Image,queue_size=10)
    self.array_pub = rospy.Publisher('floats', numpy_msg(Floats), queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/cameras/head_camera/image",Image,self.callback)

  def state_calc(self,Board):
    global state
    state = 0
    for i in range(0,6):
        state += Board[i]*(4**i)

    return state
    
  def action_calc(self,state):
    max_val = 0
    global Q_Table
    for i in range(0,6):
      if max_val <= Q_Table[state][i]:
        max_val = Q_Table[state][i]
        index_max = i
    return index_max


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    

    (rows,cols,channels) = cv_image.shape
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100,150,0])
    upper_blue = np.array([140,255,255])
    lower_green = np.array([29,86,6])
    upper_green = np.array([64,255,255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_2 = cv2.inRange(hsv, lower_green, upper_green)
    res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

    
    

    #centers can be treated as 2D arrays and to access data, use indices
    global flag
    global flag_2
    global state
    global Board_2
    global action_array
    if flag==0:
      _, contours, _ = cv2.findContours(mask.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
      cnts = cv2.findContours(mask_2.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
      center_green = None
      if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center_green = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
      
      centres = []
      for i in range(len(contours)):
        moments = cv2.moments(contours[i])
        centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
        cv2.circle(mask, centres[-1], 3, (0, 0, 0), -1)

      centers_blue = centres
      index_b1 = (0.031746032)*centers_blue[0][0]-8.698412698
      index_b1 = int(round(index_b1,0))
      index_b2 = (0.031746032)*centers_blue[1][0]-8.698412698
      index_b2 = int(round(index_b2,0))
      index_g1 = (0.031746032)*center_green[0]-8.698412698
      index_g1 = int(round(index_g1,0))
      Color_Array[index_b1] = 2
      Color_Array[index_b2] = 2
      Color_Array[index_g1] = 3
      #Color_Array has the jumbled array

      state = self.state_calc(Color_Array)
      print(state)

      
      array = [0,0,0,0,0,0]
      j = 0
      while flag_2 == 0:
        action = self.action_calc(state)
        print(action)
        action_array[j] = action
        Board_2[j] = Color_Array[action]
        Color_Array[action]=0
        j+=1
        next_state = self.state_calc(Color_Array)
        if next_state==0:
          flag_2 = 1
          print(Board_2)
          
        else:
          state = next_state
        
      
      print(Color_Array)
      flag = 1
    
    #start if condition here
    cv2.imshow("Image window", cv_image)
    #cv2.imshow('mask',mask)
    #cv2.imshow('mask_green',mask_2)
    #cv2.imshow('res',res)
    array = np.array(action_array,dtype=np.float32)
    self.array_pub.publish(array)  
    print(action_array)
    cv2.waitKey(3)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
      
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
