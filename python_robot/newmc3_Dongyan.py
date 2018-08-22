
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs
import numpy as np
import matplotlib.pyplot as plt
# from yolo import MC3YOLO
from PIL import Image
import time
import os
import sys
sys.path.append(os.path.join(os.getcwd(),'darknet/python/'))
import darknet as dn

from location import allposition
import yaml

from findBall_release import doSomething

import point_control
import velocity_control


from  checkBallNear_release import *


# added by dongyan
from geometry_msgs.msg import Twist, Point 
import math




rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
point_control.init()
velocity_control.init()
point_control.reset_odom()
#rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
velocity_control.vel_control(0.0,0.0)
print 'hello'
goal_x, goal_y=[0,0]


# car state machine
carState_begin = 'carState_begin'
carState_searchBall = 'carState_searchBall'
carState_chaseBall = 'carState_chaseBall'
carState_pushBall = 'carState_pushBall'
carState_wait = 'carState_wait'

carState = carState_begin


# variables in control
g_targetPointX, g_targetPointY = 0, 0
g_targetSpeedLinear, g_targetSpeedAngular = 0, 0


while not rospy.is_shutdown():
    global g_targetPointX, g_targetPointY, g_targetSpeedLinear, g_targetSpeedAngular

    #print "Current state: ", carState

    # state change part
    if carState == carState_begin:

        g_targetPointX, g_targetPointY = 0.1, 0.1
        (currentPoint, currentRotation) = point_control.get_odom()
        distance_error = math.sqrt(pow((currentPoint.x - g_targetPointX),2) + pow((currentPoint.y - g_targetPointY), 2))
        if distance_error < 0.05:
            carState = carState_searchBall
            print "[State change]==> from carState_begin to carState_searchBall"

    if carState == carState_searchBall:
        # if find ball
            carState = carState_chaseBall
            print "[State change]==> from carState_searchBall to carState_chaseBall"
        # if 


    if carState == carState_chaseBall:


    if carState == carState_wait:
        print "Waiting..."

    # control part
    if carState == carState_begin:
        point_control.PointMove(g_targetPointX, g_targetPointY, 0)

    if carState == carState_wait:
        velocity_control.vel_control(0, 0)
