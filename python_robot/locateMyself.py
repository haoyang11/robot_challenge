
'''
Shoot ball program.

How it works:
1. Find ball. If can find one, move to the behind of ball. If not, move back by 30cm
2. Move to the behind(30cm) of ball by 'point_control', then rotate to face front.
3. After rotate to the front, check whether the ball is in the front.
4. If the ball is in the front, move forward by 50cm and stop to find ball again. 
    If the ball angle is too large, find ball and move to the behind again.
5. If cannot find the ball, move backward by 30cm and find ball again.

6. Try to add Door. 

Dongyan, 2018.8.2

7.Used new parameter to speed it up.
8. Comment "backward time constraint"

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

import point_control2
import velocity_control

# import by dongyan
from checkBallNear_release import isBallNear
import math
from findBoundaryDistance_release import findBoundaryDistance
from checkCarNear_release import isCarNear
import motion_control

g_isBallNear = False


# import by xy
import time
import os
import sys
sys.path.append(os.path.join(os.getcwd(),'darknet/python/'))
import darknet as dn
from detect import detect



door_distance=1.15
moveback_times=0
g_dict = {"door":[],"football":[],"robot":[],"conf":[]}
net_init = True
net = None
meta = None
cam_param = None
det_flag=False
has_rev=False
np_arr = None
netInitFinished = False     # after init the net, can use the picture to find ball

net_init_top = True
net_top = None
meta_top = None
cam_param_top = None
det_flag_top =False
has_rev_top =False
np_arr_top  = None
netInitFinished_top  = False     # after init the net, can use the picture to find ball




issave = True
isshow = False

import datetime
start_t = datetime.datetime.now()
output_dir = os.path.join("./run_pic","{:%Y%m%dT%H%M}".format(start_t))
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

playGroundWidth = 1.7
playGroundLength = 2.3


def getPicture():
    global has_rev, det_flag, np_arr
    has_rev = False

    picTime = time.time()
    while (time.time()-picTime < 0.5):
        pass
    det_flag=True
    while has_rev == False:
        pass
    det_flag=False
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image_np

def findElement(img):
    global net, meta, cam_param, output_dir
    img_np = img

    t = time.time()-start_time
    # result_dict = detect(net,meta,cam_param,image_np,isshow=isshow,issave=issave,name=str(t),output_dir=output_dir)
    result_dict = detect(net,meta,cam_param,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir)

    # print "Result, door: ", result_dict['door']
    return result_dict

def getPicture_top():
    global has_rev_top, det_flag_top, np_arr_top
    has_rev_top = False

    picTime = time.time()
    while (time.time()-picTime < 0.5):
        pass
    det_flag_top=True
    while has_rev_top == False:
        pass
    det_flag_top=False
    image_np = cv2.imdecode(np_arr_top, cv2.IMREAD_COLOR)
    return image_np

def findElement_top(img):
    global net_top, meta_top, cam_param_top, output_dir
    img_np = img

    t = time.time()-start_time
    # result_dict = detect(net,meta,cam_param,image_np,isshow=isshow,issave=issave,name=str(t),output_dir=output_dir)
    result_dict = detect(net_top,meta_top,cam_param_top,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir)

    # print "Result, door: ", result_dict['door']
    return result_dict

'''
def findHorizontalPosition(leftFirst = True):
    img = getPicture()
    if leftFirst:
        rotateTo2(90)
        distnaceToLeft = findBoundaryDistance(img)
        if distnaceToLeft < 0.1:
            print "Too near to the wall, see right then."
            rotateTo2(-90)
            distnaceToRight = findBoundaryDistance(img)
            if distnaceToRight < 0.1:
                print "[Error]. findHorizontalPosition failed..."
                return
            return (playGroundWidth - distnaceToRight)
        else:
            return distnaceToLeft
    else:
        print "Cannot rotate to right now..."
'''



'''
def toCenter():
    anykey = raw_input("============= Press any key to start =========== \n")

    rotateTo2(0)

    img = getPicture()
    picInfo = findElement(img)

    x, y = 0, 0

    if len(picInfo['door']) < 1:
        print "Cannot find door"
        anykey = raw_input("============= Press any key to continue =========== \n")

    elif len(picInfo['door']) == 1:
        doorX = picInfo['door'][0][0]
        doorY = picInfo['door'][0][1]
        print "Only one pillar, at ", doorX, doorY

        anykey = raw_input("============= Press any key to continue =========== \n")

        if doorY > 0:
            print "The pillar is on the left, then turn right"
            print "Not considered yet..."

            return 

        else:
            print "The pillar is on the right, then turn left"
            h_distance = findHorizontalPosition()
            rotateTo2(0)

            deltaX = -(playGroundLength/2 - doorX)
            deltaY = -(playGroundWidth/2 - h_distance)

        print "Center point is: ", deltaX, deltaY
        anykey = raw_input("============= Press any key to continue =========== \n")
        point_control2.PointMove(deltaX, deltaY, 0)

    elif len(picInfo['door']) == 2:
        print "Find both pillars.", picInfo['door']

        x = (picInfo['door'][0][0] + picInfo['door'][1][0]) / 2 - 1.15
        y = (picInfo['door'][0][1] + picInfo['door'][1][1]) / 2
        print "Cenert is:", x, y
        anykey = raw_input("============= Press any key to continue =========== \n")
        point_control2.PointMove(x, y, 0)
    else:
        print "Error. No such conditions"

    return x, y


def toMiddleLine():
    global has_rev, det_flag, np_arr, net, meta, cam_param, output_dir

    rotateTo2(90)

    has_rev = False
    picTime = time.time()
    while (time.time()-picTime < 0.5):
        pass
    det_flag=True
    while has_rev == False:
        pass
    det_flag=False
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


    distance = findBoundaryDistance(image_np)*0.01 # to meter

    point_control2.PointMove(distance - 0.85, 0, 0)
'''

def image_callback(msg):
    global net, meta, cam_param, det_flag, has_rev, netInitFinished,np_arr 
    if netInitFinished == False:
        model_name = 'all_640'
        steps = '190000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="cam_top.yaml"
        f = open(filename)
        cam_param = yaml.load(f)
        netInitFinished = True
        
    if det_flag:
        det_flag=False
        np_arr = np.fromstring(msg.data, np.uint8)
        has_rev = True

def image_callback_top(msg):
    global  net_top, meta_top, cam_param_top, det_flag_top, has_rev_top, netInitFinished_top,np_arr_top 
    if netInitFinished_top == False:
        model_name = 'all_640'
        steps = '190000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net_top = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta_top = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="cam_top.yaml"
        f = open(filename)
        cam_param_top = yaml.load(f)
        netInitFinished_top = True
        
    if det_flag_top:
        det_flag_top=False
        np_arr_top = np.fromstring(msg.data, np.uint8)
        has_rev_top = True
        # print np_arr

'''
def rotateTo(angle):     # rotate to a certain angle, with default speed 0.4
    rate = rospy.Rate(100)
    rpy=point_control2.get_imu()
    angle_error = angle - rpy[2]
    if(angle_error > 180):
        angle_error -= 360
    elif(angle_error < -180):
        angle_error += 360

    angle_error_old = 0

    while abs(angle_error) > 5:

        rpy=point_control2.get_imu()
        angle_error_old = angle_error   # pd
        angle_error = angle - rpy[2]
        if(angle_error > 180):
            angle_error -= 360
        elif(angle_error < -180):
            angle_error += 360

        speed = angle_error * 0.01
        if speed > 0:
            speed = min(speed, 2.0) # upperspeed, 2
        else:
            speed = max(speed, -2.0)
        velocity_control.vel_control(0, speed)    # low speed to rotate to front
        rate.sleep()
    velocity_control.vel_control(0, 0)
'''


def rotateTo2(angle):     # rotate to a certain angle, with default speed 0.4
    rate = rospy.Rate(100)
    # global rate
    rpy=point_control2.get_imu()
    angle_error = angle - rpy[2]
    if(angle_error > 180):
        angle_error -= 360
    elif(angle_error < -180):
        # print "angle_error,", angle_error
        angle_error += 360
        # print "angle_error,", angle_error

    # print "<rotateTo> Input angle:", angle, "current angle: ",rpy[2], "angle error:", angle_error
    # print "angle_error: ", angle_error,"angle_now: ", rpy[2]

    angle_error_old = 0

    while abs(angle_error) > 5:

        rpy=point_control2.get_imu()
        # print "angle_error: ", angle_error,"angle_now: ", rpy[2]
        angle_error_old = angle_error   # pd
        angle_error = angle - rpy[2]
        if(angle_error > 180):
            angle_error -= 360
        elif(angle_error < -180):
            angle_error += 360

        # speed = angle_error * 0.04 + (angle_error - angle_error_old) * 0.02
        # speed = max(angle_error*0.02, 0.4)  #lowerspped, 0.4
        speed = angle_error * 0.03
        if speed > 0:
            speed = min(speed, 2.0) # upperspeed, 2
        else:
            speed = max(speed, -2.0)
        velocity_control.vel_control(0, speed)    # low speed to rotate to front

        # print "<rotateTo> Stopped: ",rpy[2], "angle error:", angle_error


        # print "angle_error", angle_error
        rate.sleep()
    # print "<rotateTo> Rotate finished at angle: ", rpy[2]

    velocity_control.vel_control(0, 0)

'''
def isBackline(dict0):

    return False

    global moveback_times,door_distance
    if len(dict0['door'])>=1:
        if len(dict0['door'])>=2:
            if dict0['door'][0][0]<dict0['door'][1][0]:
                door_distance=dict0['door'][0][0]
            else:
                door_distance=dict0['door'][1][0]
        else:
            door_distance=dict0['door'][0][0]
        moveback_times=0
    if (moveback_times+1)*0.3 > (2.3-door_distance):
        print "Now:door_distance=",door_distance,"and i have ",moveback_times,"backs","so backline"
        return True
    else:
        print "Now:door_distance=",door_distance,"and i have ",moveback_times,"backs","so not backline"
        return False
'''


#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
# point_control2.init()
# velocity_control.init()
motion_control.init()
# point_control2.reset_odom()
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
rospy.Subscriber("/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage, image_callback_top)
goal_x, goal_y=[0,0]
# ---------------------------------init--------------------------------------------


state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'
state_wait = 'state_wait'
state_unexpected = 'state_unexpected'

carState = state_begin       # begin with "findBall"

targetPointX, targetPointY = 0,0
print "================   waiting for odom to reset ========================="
anykey = raw_input("============= Press any key to start =========== \n")
print "================   a key is pressed          ========================="
start_time = time.time()


isMyHalf = True
moveBackCount = 0

while not rospy.is_shutdown():

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        motion_control.PointMove(0.1, -0.1, 0)
        carState = state_findBall

    elif carState == state_findBall:
        print "---------------------  <state> find ball ---------------------"
        anykey = raw_input("============= Press any key to continue =========== \n")
        motion_control.rotateTo2(0)        # to face the front
        img = getPicture_top()
        result = findElement_top(img)

        if len(result['football']) > 1:
            ballX, ballY = result['football']
            print "Find football at: ", ballX, ballY
            if abs(ballY) < 0.1:            # the ball is in the front
                print "Robot is facing the ball."
                motion_control.PointMove(ballX + 0.3, 0)
            else:
                print "Robot is not facing the ball, try to go the behind."
                motion_control.PointMove(ballX - 0.3, ballY)
            # carState remains to be state_findBall
        else:
            print "Cannot find football."
            carState = state_changePosition

    elif carState == state_changePosition:

        '''
        if isMyHalf == False:
            moveBackCount = 0           # if not my half, can move back reset.
        '''

        print "---------------------  <state> change position ---------------------"
        anykey = raw_input("============= Press any key to continue =========== \n")
        motion_control.rotateTo2(0)
        img = getPicture_top()

        if isCarNear(img):
            print "Car is near. Do not move at all."
            anykey = raw_input("============= Press any key to continue =========== \n")
            carState = state_findBall
            continue

        # now cannot find ball, and car is not in the front
        print "No ball, no car around, so try to find door"
        result = findElement_top(img)
        if len(result['door']) == 0:
            print "Cannot find door. Move back safely..."
            motion_control.rotateTo2(0)
            motion_control.PointMove(-0.2, 0, 0)
            '''
            # print "[Error] Cannot find the door."
            # carState = state_unexpected
            if isMyHalf == False:
                print "--> is not my half, can move back now."
                rotateTo2(0)
                point_control2.PointMove(-0.2, 0, 0)
            else:
                if moveBackCount < 3:
                    print "Move back count is: ", moveBackCount, " can move back again."
                    rotateTo2(0)
                    point_control2.PointMove(-0.2, 0, 0)
                    moveBackCount += 1
                    # carState = state_findBall
                else:
                    print "Move back for 3 times. Cannot move back again."
            '''

        else:
            # doorInfo = result['door']
            doorX, doorY = 0, 0
            if len(result['door']) == 1:
                doorX, doorY = result['door'][0][0], result['door'][0][1]
                print "Find one pillar at:", doorX, doorY
            else:
                d1_x, d1_y = result['door'][0][0], result['door'][0][1]
                d2_x, d2_y = result['door'][1][0], result['door'][1][1]
                print "Find two pillar, at:", d1_x, d1_y,", 2: ", d2_x, d2_y
                # doorX = (d1_x + d2_x) * 0.5
                doorX = min(d1_x, d2_x)
                doorY = (d1_y + d2_y) * 0.5

            '''
            if doorX > 1.15:
                isMyHalf = True
            else:
                isMyHalf = False
            '''

            if doorX < 1.5:
                print "The door is not far away, can move back."
                motion_control.rotateTo2(0)
                motion_control.PointMove(-0.2, 0, 0)
                # carState = state_findBall
            else:
                print "The door is vary far, cannot move back again. Try to find ball again."
                # carState = state_findBall

        # always change to "findball" state after changing position
        carState = state_findBall

    elif carState == state_unexpected:
        print "==> [Error]: state_unexpected."
        while True:
            pass

    else:
        print "==> [Error]: error state"







