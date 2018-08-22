'''

No communication.
Master
Onlybottom

8.17 8:20

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


from sensor_msgs.msg import Image 
from std_msgs.msg import Empty,UInt8MultiArray, String
import socket


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
cv_image = None
# top camera's information.
net_init_top = True
net_top = None
meta_top = None
cam_param_top = None
det_flag_top =False
has_rev_top =False
np_arr_top  = None
netInitFinished_top  = False
cv_image_top = None

issave = True
isshow = False


empty_msg=Empty()
start_time = time.time()


import datetime
start_t = datetime.datetime.now()
output_dir_base = os.path.join("./run_pic","{:%Y%m%dT%H%M}".format(start_t))
if not os.path.exists(output_dir_base):
    os.makedirs(output_dir_base)
output_dir_top = os.path.join(output_dir_base,'top')
if not os.path.exists(output_dir_top):
    os.makedirs(output_dir_top)
output_dir = os.path.join(output_dir_base,'bottom')
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# def getPicture():
#     print "Get Picture once..."
#     global has_rev, det_flag, np_arr,ask_camera
#     has_rev = False
#     # picTime = time.time()
#     # while (time.time()-picTime < 0.01):
#     #     pass
#     det_flag=True
#     ask_camera.publish(empty_msg)      # new added
#     while has_rev == False:
#         pass
#     det_flag=False
#     return cv_image

def findElement(img):
    global net, meta, cam_param, output_dir
    img_np = img
    t = time.time()-start_time
    result_dict = detect(net,meta,cam_param,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir)
    return result_dict

# def getPicture_top():
#     print "Get Picture Top once..."
#     global has_rev_top, det_flag_top, np_arr_top,ask_camera_top
#     has_rev_top = False
#     # picTime = time.time()
#     # while (time.time()-picTime < 0.01):
#     #     pass
#     det_flag_top=True
#     ask_camera_top.publish(empty_msg)
#     while has_rev_top == False:
#         pass
#     det_flag_top=False
#     return cv_image_top

def findElement_top(img):
    global net_top, meta_top, cam_param_top, output_dir_top
    img_np = img
    t = time.time()-start_time
    result_dict = detect(net_top,meta_top,cam_param_top,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir_top)
    return result_dict


cv_image = None

def image_callback(msg):
    print "Image Callback once..."
    global det_flag, has_rev, cv_image
    if det_flag:
        det_flag=False
        np_narray = np.fromstring(msg.data, np.uint8)
        np_narray = np_narray.reshape((-1,1))
        cv_image = cv2.imdecode(np_narray,cv2.IMREAD_COLOR)
        has_rev = True


def image_callback_top(msg):
    print "Image Callback  Top once..."
    global  det_flag_top, has_rev_top, cv_image_top
    if det_flag_top:
        det_flag_top=False
        np_narray = np.fromstring(msg.data, np.uint8)
        np_narray = np_narray.reshape((-1,1))
        cv_image_top = cv2.imdecode(np_narray,cv2.IMREAD_COLOR)
        has_rev_top = True


def findBallDoorTogether():

    print "*************************************************"
    returnDict = {"football":[], "door":[]}

    global has_rev, det_flag, np_arr, ask_camera
    # global has_rev_top, det_flag_top, np_arr_top,ask_camera_top

    global net, meta, cam_param, output_dir, cv_image
    global net_top, meta_top, cam_param_top, output_dir_top, cv_image_top

    has_rev, has_rev_top = False, False
    det_flag, det_flag_top = True, True

    ask_camera.publish(empty_msg)

    t = time.time()

    while has_rev == False:
        pass
    det_flag=False

    return cv_image



def camera_init():
    global  net_top, meta_top, cam_param_top
    global net, meta, cam_param

    # bottom camera
    model_name = 'all_640_xy'
    steps = '170000'
    base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
    net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
    meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
    filename="cam_param/"+tb_name+"_bottom_new.yaml"
    f = open(filename)
    cam_param = yaml.load(f)

    # top camera
    model_name = 'door'
    steps = '250000'
    base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
    net_top = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
    meta_top = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
    filename="cam_param/"+tb_name+"_top.yaml"
    f = open(filename)
    cam_param_top = yaml.load(f)
    netInitFinished_top = True


#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)
rospy.Subscriber("/tb"+tb_name+"/pub_camera",UInt8MultiArray,image_callback)
ask_camera=rospy.Publisher("/tb"+tb_name+"/ask_camera", Empty,queue_size=1)
# rospy.Subscriber("/tb"+tb_name+"/pub_camera_top",UInt8MultiArray,image_callback_top)
# ask_camera_top=rospy.Publisher("/tb"+tb_name+"/ask_camera_top", Empty,queue_size=1)

rate = rospy.Rate(10)
bridge = CvBridge()
motion_control.init()

camera_init()
# ---------------------------------init--------------------------------------------

state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'


anykey = raw_input("============= Press any key to start =========== \n")
print "================  a key is pressed. Go go go! ========================="

carState = state_begin       # begin with "findBall"
isNearBaseLine = True
navigateCount = 0


while not rospy.is_shutdown():
    global masterFindBall, isSlaverAttacking, isNearBaseLine,navigateCount

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        # motion_control.PointMove(0.9, -0.55, 0)      # begin to move to a point.
        # motion_control.PointMove(0.5, 0, 0)

        # motion_control.PointMove(1.2, -0.65, 0)

        motion_control.PointMove(0.4, -0.4, 0)      # begin to move to a point.
        # motion_control.PointMove(0.5, 0, 0)
        isNearBaseLine = True
        carState = state_findBall

    else:

        if carState == state_findBall:

            bottomImg = findBallDoorTogether()

            # print "---------------------  <state> find ball ---------------------"
            if isBallNear(bottomImg):
                isNearBaseLine = False
                motion_control.PointMove(0.4, 0, 0)

            result = findElement(bottomImg)
            if len(result['football']) > 1:
                masterFindBall = True
                ballX, ballY = result['football']
                if ballX > 0.2:
                    # print "Move froward a little. NOT near baseline now."
                    isNearBaseLine = False
                print "Find football at: ", ballX, ballY

                if abs(ballY) < 0.1:            # the ball is in the front
                    print "Robot is facing the ball. Move forward."
                    motion_control.PointMove(ballX + 0.4, 0, 0)
                else:
                    # print "Robot is NOT facing the ball, try to go to the behind."
                    if ballY > 0:
                        motion_control.PointMove(ballX - 0.2, ballY + 0.04, 0)
                    else:
                        motion_control.PointMove(ballX - 0.2, ballY - 0.04, 0)
                    
            else:
                # print "No ball found by yolo, ",
                if isCarNear(bottomImg):
                    print "Car! Car! Car! Car! Car! Car!"
                    continue
                else:
                    # print "a car is NOT near. Change to moveback"
                    carState = state_changePosition

        if carState == state_changePosition:
            # print "---------------------  <state> change position ---------------------"
            result = findElement_top(bottomImg)
            if len(result['door']) == 0:
                print "<DOOR> Cannot find door. Move back safely..."
                # motion_control.rotateTo2(0)
                motion_control.PointMove(-0.3, 0, 0)
            else:
                doorX, doorY = 0, 0
                if len(result['door']) == 1:
                    doorX, doorY = result['door'][0][0], result['door'][0][1]
                    # print "<DOOR> Find one pillar at:", doorX, doorY
                else:
                    d1_x, d1_y = result['door'][0][0], result['door'][0][1]
                    d2_x, d2_y = result['door'][1][0], result['door'][1][1]
                    print "<DOOR> Find two pillars, at:", d1_x, d1_y,", 2: ", d2_x, d2_y
                    doorX = (d1_x + d2_x) * 0.5
                    # doorX = max(d1_x, d2_x)     # use the more near one to check the distance
                    doorY = (d1_y + d2_y) * 0.5 # doorY is not used so far.


                # master: 1.7-1.9.
                if doorX < 1.7:
                    if isNearBaseLine == True:
                        # print "No move back because of near BaseLine"
                        pass
                    else:
                        print "<BaseLine> The door is NOT far away, can move back."
                        # motion_control.rotateTo2(0)
                        motion_control.PointMove(-0.15, 0, 0)
                elif doorX > 1.9:
                    print "<BaseLine> Almost myhalf middle line. Move forward a little"
                    # motion_control.rotateTo2(0)
                    motion_control.PointMove(0.2, 0, 0)
                else:
                    print "<BaskLine> Just in position, do not move back or forward. Find ball again."
                    pass

                if doorX > 1.7:
                    isNearBaseLine = True

                if isNearBaseLine == True:

                    print "Move around the line ..., Times: ", navigateCount
                    if navigateCount == 0:
                        pass
                    elif navigateCount == 1:
                        motion_control.rotateTo2(90)
                        motion_control.PointMove(0.5, 0, 0)
                    elif navigateCount == 2 or navigateCount == 3:
                        motion_control.rotateTo2(-90)
                        motion_control.PointMove(0.5, 0, 0)
                    elif navigateCount == 4:
                        motion_control.rotateTo2(90)
                        motion_control.PointMove(0.5, 0, 0)
                        navigateCount = 0
                    else:
                        print "Error"
                        key = raw_input("Error---\n")
                    navigateCount += 1
                    
                else:
                    navigateCount = 0
                    
            # always change to "findball" state after changing position
            carState = state_findBall



