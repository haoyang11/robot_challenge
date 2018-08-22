
'''
Defend  program

1. Move to the door.
2. Find ball, if find, check the distance:
                        if the distance is near, attack. else, rotate to face the ball
3. If cannot find the ball, face to left, middle, right to find the ball.


Dongyan 2018.8.2

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

np_arr = None
netInitFinished = False     # after init the net, can use the picture to find ball

issave = True
isshow = False
start_time = time.time()
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

# ============= function by dongyan =================


def rotateTo(angle, speed=0.4):     # rotate to a certain angle, with default speed 0.4

    global rate
    rpy=point_control.get_imu()
    angle_error = angle - rpy[2]
    print "<rotateTo> Input angle:", angle, "current angle: ",rpy[2], "angle error:", angle_error
    # print "angle_error: ", angle_error,"angle_now: ", rpy[2]

    while abs(angle_error) > 5:

        rpy=point_control.get_imu()
        # print "angle_error: ", angle_error,"angle_now: ", rpy[2]
        angle_error = angle - rpy[2]
        if angle_error > 0:
            velocity_control.vel_control(0, speed)    # low speed to rotate to front
        else:
            velocity_control.vel_control(0, -speed)

        # print "angle_error", angle_error
        rate.sleep()
    print "<rotateTo> Rotate finished at angle: ", rpy[2]

    velocity_control.vel_control(0, 0)

def isDangerous(ballX, ballY):      # check whether need to attack

    global rate, carState
    distance = math.sqrt(pow(ballX, 2)+pow(ballY, 2))
    # print "Distance is:", distance, ", angd Anlge is: ",angle

    # can use angle and distance to judge attacking...
    if distance > 1:
        print "Ball is not near."
        return False
        # rotateTo(angle)
    else:
        print "Ball is near 1m."
        return True

# ============= function by dongyan =================


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


state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_attack = 'state_attack'
state_lookAround = 'state_lookAround'


state_moveToPoint = 'state_moveToPoint'
state_moveBack = 'state_moveBack'
state_rotateToFront = 'state_rotateToFront'

print "================   waiting for odom to reset ========================="
anykey = raw_input("============= Press any key to start =========== \n")
print "================   a key is pressed          ========================="



carState = state_begin       # begin with "findBall"

lookAroundCount = 0

while not rospy.is_shutdown():
    global netInitFinished
    global det_flag, has_rev, np_arr, rate
    global g_dict, net, meta, cam_param, net_init, det_flag, has_rev, start_time

    if netInitFinished == False:    # wait for net to finish init
        continue

    if carState == state_begin:
        print "[state] begin. Move to my door."
        point_control.PointMove(0.3, 0, 0)
        rotateTo(75,0.6)
        # anykey = raw_input("============= Press any key to continue =========== \n")
        rotateTo(90,0.3)
        point_control.PointMove(0.5, 0, 0)
        rotateTo(25,0.6)
        rotateTo(0,0.3)
        point_control.PointMove(-0.25, 0, 0)
        cnt = 0
        while 1:
            print "Rotate ", cnt, " circles......"
            rotateTo(0,0.3)
            rotateTo(90,0.3)
            rotateTo(179,0.3)
            rotateTo(-90,0.3)



        carState = state_findBall

    elif carState == state_findBall:
        print "[state] find ball."
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # print "image_np:",image_np
        result_dict = detect(net,meta,cam_param,image_np,isshow=False,issave=False,name="str(t)")
        # result_dict["football"]=[0.5,0.2]

        # find ball. wait or attack
        if len(result_dict["football"])>1: 
            x, y = result_dict['football']
            print "Find ball at: ", x, y
            if isDangerous(x, y):
                print "Attack..."
                carState = state_attack
            else:
                print "Not dangerous. rotate to face ball and wait..."
                angle = math.atan2(y, x) * 180 / 3.1415926   
                # anykey = raw_input("============= Press any key to continue =========== \n")
                print "Face ball angle: ", angle
                rotateTo(angle, 0.3)
                print "Rotate finished..."
                # anykey = raw_input("============= Press any key to continue =========== \n")
               
        else:
            print "Cannot find ball."
            carState = state_lookAround

    elif carState == state_lookAround:
        print "[state] look around..."
        rotateTo((2-lookAroundCount) * 30, 0.3)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # print "image_np:",image_np
        result_dict = detect(net,meta,cam_param,image_np,isshow=False,issave=False,name="str(t)")
        if len(result_dict["football"])>1: 
            print "find ball when looking around. --> change to find ball."
            carState = state_findBall
            lookAroundCount = 0         # reset the counter
        else:
            print "cannot find ball when looking around."
            lookAroundCount += 1
            if lookAroundCount == 5:
                lookAroundCount = 0     # reset 
    elif carState == state_attack:

        print "[state] Attacking!..."

        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        result_dict = detect(net,meta,cam_param,image_np,isshow=False,issave=False,name="str(t)")

        if len(result_dict["football"])>1: 
            x, y = result_dict["football"]
            angle = math.atan2(y, x) * 180 / 3.1415926
            rotateTo(angle, 0.2)
            print "Attacking and rush to the ball."

            point_control.PointMove(0.2, 0, 0)
            # point_control.PointMove()
            while True:
                pass
                # print "Watiing...."

        


