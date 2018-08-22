
'''
Shoot ball program.

How it works:
1. Find ball. If can find one, move to the behind of ball. If not, move back by 30cm
2. Move to the behind(30cm) of ball by 'point_control', then rotate to face front.
3. After rotate to the front, check whether the ball is in the front.
4. If the ball is in the front, move forward by 50cm and stop to find ball again. 
    If the ball angle is too large, find ball and move to the behind again.
5. If cannot find the ball, move backward by 30cm and find ball again.


Dongyan, 2018.8.2
'''

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs
import numpy as np
import matplotlib.pyplot as plt
# from yolo import MC3YOLO
from PIL import Image

from location import allposition
import yaml

import point_control
import velocity_control

# import by dongyan
from checkBallNear_release import isBallNear
import math

g_isBallNear = False


# import by xy
import time
import os
import sys
sys.path.append(os.path.join(os.getcwd(),'darknet/python/'))
import darknet as dn
from detect import detect



g_dict = {"door":[],"football":[],"robot":[],"conf":[]}
net_init = True
net = None
meta = None
cam_param = None



det_flag=False
has_rev=False

np_arr = 0
netInitFinished = False     # after init the net, can use the picture to find ball
def image_callback(msg):
    global g_dict, net, meta, cam_param, net_init, det_flag, has_rev, netInitFinished,np_arr 

    if netInitFinished == False:
        model_name = 'all_640'
        steps = '180000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="cam_top.yaml"
        f = open(filename)
        cam_param = yaml.load(f)
        

    np_arr = np.fromstring(msg.data, np.uint8)
    # print np_arr
    netInitFinished = True

#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
point_control.init()
velocity_control.init()
point_control.reset_odom()
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
goal_x, goal_y=[0,0]
# ---------------------------------init--------------------------------------------



state_findBall = 'state_findBall'
state_moveToPoint = 'state_moveToPoint'
state_moveBack = 'state_moveBack'
state_rotateToFront = 'state_rotateToFront'

carState = state_findBall       # begin with "findBall"

targetPointX, targetPointY = 0,0
print "================   waiting for odom to reset ========================="
anykey = raw_input("============= Press any key to start =========== \n")
print "================   a key is pressed          ========================="

while not rospy.is_shutdown():
    global netInitFinished
    global det_flag, has_rev, np_arr, rate
    global g_dict, net, meta, cam_param, net_init, det_flag, has_rev
    
    if netInitFinished == False:    # wait for net to finish init
        continue

    if carState == state_findBall:
        print "[state] findBall. Try to find ball"
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print "image_np:",image_np
        result_dict = detect(net,meta,cam_param,image_np,isshow=False,issave=False,name="str(t)")
        result_dict["football"]=[0.5,0.2]
        if len(result_dict["football"])>1: 
            carState = state_moveToPoint
            targetPointX, targetPointY = result_dict['football']
            print "Ball position: ", targetPointX, targetPointY

        else:
            print "Cannot find ball."
            carState = state_moveBack
        
    else:

        if carState == state_moveToPoint:
            print "[state] moveToPoint. "
            print "POS:",[targetPointX,targetPointY]
            point_control.PointMove(targetPointX - 0.3, targetPointY, 0)
            print "Stopped at target point."
            carState = state_rotateToFront

        else:
            if carState == state_rotateToFront:
                print "[state] rotate to front. "

                rpy=point_control.get_imu()
                angle_error = 0 - rpy[2]

                while abs(angle_error) > 5:

                    rpy=point_control.get_imu()
                    angle_error = 0 - rpy[2]
                    if angle_error > 0:
                        velocity_control.vel_control(0, 0.4)    # low speed to rotate to front
                    else:
                        velocity_control.vel_control(0, -0.4)

                    print "angle_error", angle_error
                    rate.sleep()
                velocity_control.vel_control(0, 0)
                print "Finaly error: ", angle_error

                
                # get ball position:
                image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                result_dict = detect(net,meta,cam_param,image_np,isshow=False,issave=False,name="str(t)")
        
                ballAngleError = 100
                if len(result_dict["football"])>1:
                    targetPointX, targetPointY = result_dict['football']
                    ballAngleError = abs(atan2(targetPointY, targetPointX) * 180 / 3.1415926)
                else:
                    print "[Error] Cannot find ball after rotate."

                if ballAngleError < 5:
                    print "Angle error is small. Move forward."
                    point_control.PointMove(0.5, 0, 0)
                    carState = state_findBall
                else:
                    print "Angle error is large, = ", ballAngleError
                    carState = state_findBall

            else:
                if carState == state_moveBack:
                    print "[state] state_moveBack"
                    point_control.PointMove(-0.2, 0, 0)
                    carState = state_findBall
                else:
                    print "[Error] Unexpected state"

