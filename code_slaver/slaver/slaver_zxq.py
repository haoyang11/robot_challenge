'''

Use two camera to find ball and car.

Procedure:
1. Go to begin point first. (0.85, 0.5) or (0.85, -0.5), then change to 'state_findBall'
2. In 'state_findBall':
    - Use HSV to find ball first. (function: isBallNear)
        - If the ball is in the front (abs(deltaY)<0.1), move forward by 20cm. Then continue 'state_findBall' again.
    - Use yolo to find ball then.
        - If the ball is in the front (abs(deltaY)<0.1), move forward by (ballX <from yolo and camera's parameters> + 30) cm. Then contiunue 'state_findBall' again
        - If the ball is not in the front, move to the 20cm behind the ball, using motion_control.PointMove()
        - If cannot find the ball, jump to 'state_changePosition'
3. In 'state_changePosition':
    - Use function 'isCarNear' to check whether a car is in the front.
        - If a car is in the front, do not move at all. Then continue 'state_findBall' (waiting for that car to move away)
    - Use yolo to find the door.
        - If cannot find the door, must be near the door, so move back safely by 20cm
        - If find the door, one pillar or two, get the distance.
            - If the distance is larger than a certain value (1.5 by defualt), cannot move back again. Still to 'state_findBall'
            - If the distance is smaller than that value, move back by 20cm. Still to 'state_findBall'

Dongyan 2018.8.8

zxq change ros name 2018.8.8


New in 8.9
1. Use Slaver identification to move.
2. If no response from master, do not move. But sometimes error occurs.
3. Problem 2: picture propogation time is too long. Cannot run very fast.


4. add move back constrain.
'''

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs
import std_msgs
import numpy as np
from location import allposition
import yaml

# import by dongyan
import math
from checkBallNear_release import isBallNear
from checkCarNear_release import isCarNear
import motion_control


# import by xy
import time
import os
import sys
sys.path.append(os.path.join(os.getcwd(),'darknet/python/'))
import darknet as dn
from detect import detect


tb_name=str(os.getenv('TB_NAME'))

g_dict = {"door":[],"football":[],"robot":[],"conf":[]}
# bottom camera's information
net_init = True
net = None
meta = None
cam_param = None
det_flag=False
has_rev=False
np_arr = None
netInitFinished = False     
# top camera's information.
net_init_top = True
net_top = None
meta_top = None
cam_param_top = None
det_flag_top =False
has_rev_top =False
np_arr_top  = None
netInitFinished_top  = False

issave = True
isshow = False


import datetime
start_t = datetime.datetime.now()
output_dir_base = os.path.join("./run_pic","{:%Y%m%dT%H%M}".format(start_t))
if not os.path.exists(output_dir_base):
    os.makedirs(output_dir_base)
output_dir_top = os.path.join(output_dir_base,'top')
output_dir = os.path.join(output_dir_base,'bottom')
if not os.path.exists(output_dir_top):
    os.makedirs(output_dir_top)
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# get bottom camera's picture
def getPicture():
    global has_rev, det_flag, np_arr

    has_rev = False
    sub = rospy.Subscriber("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)

    picTime = time.time()
    #   delay should be match picture propogation time.
    while (time.time()-picTime < 0.5):
        pass
    det_flag=True
    while has_rev == False:
        pass
    sub.unregister()

    det_flag=False
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image_np

# use bottom camera's paramter to find element
def findElement(img):
    global net, meta, cam_param, output_dir
    img_np = img
    t = time.time()-start_time
    result_dict = detect(net,meta,cam_param,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir)
    return result_dict

# top camera's function.
def getPicture_top():
    global has_rev_top, det_flag_top, np_arr_top
    has_rev_top = False
    sub = rospy.Subscriber("/tb"+tb_name+"/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage, image_callback_top)

    picTime = time.time()
    #   delay should be match picture propogation time.
    while (time.time()-picTime < 0.5):
        pass
    det_flag_top=True
    while has_rev_top == False:
        pass
    sub.unregister()

    det_flag_top=False
    image_np = cv2.imdecode(np_arr_top, cv2.IMREAD_COLOR)
    return image_np

def findElement_top(img):
    global net_top, meta_top, cam_param_top, output_dir_top
    img_np = img
    t = time.time()-start_time
    result_dict = detect(net_top,meta_top,cam_param_top,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir_top)
    return result_dict


def image_callback(msg):
    global net, meta, cam_param, det_flag, has_rev, netInitFinished,np_arr 
    if netInitFinished == False:
        model_name = 'all_640_xy'
        steps = '140000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="./cam_param/"+tb_name+"_bottom.yaml"
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
        model_name = 'door'
        steps = '230000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net_top = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta_top = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="./cam_param/"+tb_name+"_top.yaml"
        f = open(filename)
        cam_param_top = yaml.load(f)
        netInitFinished_top = True
    if det_flag_top:
        det_flag_top=False
        np_arr_top = np.fromstring(msg.data, np.uint8)
        has_rev_top = True
        # print np_arr


''' 
# use computer to control movement. Not recommand.
def rotateTo2(angle):     # rotate to a certain angle, with default speed 0.4
    rate = rospy.Rate(100)
    # global rate
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


# new publisher by dongyan

slaverFindBall = False
slaverInstruction = 'noResponse'

def slaver_publish():
    global slaverFindBall,pub_slaver
    slaverInstruction = 'noResponse'

    if slaverFindBall == True:
        pub_slaver.publish(True)
    else:
        pub_slaver.publish(False)


def master_callback(msg):
    global slaverInstruction
    if msg.data == 'attack':
        print "<CALL BACK> slaver received 'attack' instruction now!"
        slaverInstruction = 'attack'
    elif msg.data == 'standby':
        print "<CALL BACK> slaver received 'standby' instruction now!"
        slaverInstruction = 'standby'
    else:
        print "<CALL BACK> [ERROR]."
        anykey = raw_input("Error.\n")



#--------------------------------- init--------------------------------------------
rospy.init_node("tb"+tb_name+'mc3_main', anonymous=True)

rate = rospy.Rate(10)
bridge = CvBridge()
motion_control.init()
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage, image_callback_top)
goal_x, goal_y=[0,0]
getPicture_top()
getPicture()
pub_slaver = rospy.Publisher("isSlaverFindBall",std_msgs.msg.Bool,queue_size=1)
rospy.Subscriber('masterInstruction',std_msgs.msg.String,master_callback)

# ---------------------------------init--------------------------------------------


state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'





anykey = raw_input("============= Press any key to start =========== \n")
print "================  a key is pressed. Go go go! ========================="

start_time = time.time()
carState = state_begin       # begin with "findBall"

g_isNearBaseLine = False

while not rospy.is_shutdown():

    global slaverFindBall, slaverInstruction, g_isNearBaseLine

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        motion_control.PointMove(0.3, 0.4, 0)      # begin to move to a point.

        # rotate to wait
        motion_control.rotateTo2(90)
        motion_control.rotateTo2(179)
        motion_control.rotateTo2(-90)
        motion_control.rotateTo2(0)

        motion_control.rotateTo2(90)
        motion_control.rotateTo2(179)
        motion_control.rotateTo2(-90)
        motion_control.rotateTo2(0)

        carState = state_findBall

    elif carState == state_findBall:
        print "---------------------  <state> find ball ---------------------"
        # anykey = raw_input("============= Press any key to continue =========== \n")

        motion_control.rotateTo2(0)        # to face the front
        img = getPicture()

        if isBallNear(img):
            slaverFindBall = True
            slaver_publish()
            while slaverInstruction == 'noResponse':
                print "Waiting for response -",
            print ""

            if slaverInstruction == 'attack':
                print "<----------->    [ SLAVER ]   ATTACK    <--------------->"
                motion_control.PointMove(0.4, 0, 0)
                continue
            elif slaverInstruction == 'standby':
                print "<----------->    [ SLAVER ]   STANDYBY    <--------------->"
                continue
            else:
                print "[ERROR] at slaverInstruction"
                anykey = raw_input("Error.\n")

        print "The ball is NOT near -- by HSV. Use yolo now."
        result = findElement(img)
        if len(result['football']) > 1:

            slaverFindBall = True
            slaver_publish()
            while slaverInstruction == 'noResponse':
                print "Waiting for response -",
            print ""

            ballX, ballY = result['football']
            print "Find football at: ", ballX, ballY
            # anykey = raw_input("============= Press any key to Move to ball... =========== \n")
            if slaverInstruction == 'attack':
                print "<----------->    [ SLAVER ]   ATTACK    <--------------->"
                if abs(ballY) < 0.1:            # the ball is in the front
                    print "Robot is facing the ball. Move forward."
                    motion_control.PointMove(ballX + 0.4, 0, 0)
                    g_isNearBaseLine = False
                else:
                    print "Robot is NOT facing the ball, try to go to the behind."
                    motion_control.PointMove(ballX - 0.2, ballY, 0)
                    if ballX > 0.5:
                        g_isNearBaseLine = False
                        
            elif slaverInstruction == 'standby':
                print "<----------->    [ SLAVER ]   STANDYBY    <--------------->"
                continue
            else:
                print "[ERROR] at slaverInstruction"
                anykey = raw_input("Error.\n")
            # carState remains to be state_findBall

        else:
            slaverFindBall = False
            slaver_publish()
            print "No ball fond by yolo, ",
            if isCarNear(img):
                print "a car is near. Do not move back."
                continue
            else:
                print "no car near. Change to moveBack"
                carState = state_changePosition

    elif carState == state_changePosition:

        print "---------------------  <state> change position ---------------------"
        # anykey = raw_input("============= Press any key to continue =========== \n")
        motion_control.rotateTo2(0)
        img = getPicture_top()      # get top camera's picture.
        result = findElement_top(img)
        if len(result['door']) == 0:
            print "<DOOR> Cannot find door. Move back safely..."
            motion_control.rotateTo2(0)
            motion_control.PointMove(-0.2, 0, 0)
        else:
            # doorInfo = result['door']
            doorX, doorY = 0, 0
            if len(result['door']) == 1:
                doorX, doorY = result['door'][0][0], result['door'][0][1]
                print "<DOOR> Find one pillar at:", doorX, doorY
            else:
                d1_x, d1_y = result['door'][0][0], result['door'][0][1]
                d2_x, d2_y = result['door'][1][0], result['door'][1][1]
                print "<DOOR> Find two pillars, at:", d1_x, d1_y,", 2: ", d2_x, d2_y
                # doorX = (d1_x + d2_x) * 0.5
                doorX = max(d1_x, d2_x)     # use the more near one to check the distance
                doorY = (d1_y + d2_y) * 0.5 # doorY is not used so far.
            if doorX > 1.8:
                print "<BaseLine> Almost near baseline. Cannot move back"
                g_isNearBaseLine = True
            else:
                if g_isNearBaseLine == True:
                    print "<BaseLine> Cannot move back because of g_isNearBaseLine is set."
                else:
                    print "<BaseLine> Move back..."
                    motion_control.rotateTo2(0)
                    motion_control.PointMove(-0.2, 0, 0)

            '''
            if doorX < 1.7:
                print "<BaseLine> The door is NOT far away, can move back."
                if g_isNearBaseLine == True:
                    print "<BaseLine> No move back because the g_isNearBaseLine is true."
                else:
                    motion_control.rotateTo2(0)
                    motion_control.PointMove(-0.2, 0, 0)

            elif doorX > 2:
                print "<BaseLine> Almost near baseline. Move forward a little"
                g_isNearBaseLine = True
                motion_control.rotateTo2(0)
                motion_control.PointMove(0.2, 0, 0)
            else:
                print "<BaskLine> Just in position, do not move at all. Find ball again."
            '''
        # always change to "findball" state after changing position
        carState = state_findBall

    else:
        print "==> [Error]: error state"

