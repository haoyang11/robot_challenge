'''

Two car use own information to attack. 
No communication.

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
if not os.path.exists(output_dir_top):
    os.makedirs(output_dir_top)
output_dir = os.path.join(output_dir_base,'bottom')
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# get bottom camera's picture
def getPicture():
    global has_rev, det_flag, np_arr
    has_rev = False

    sub=rospy.Subscriber("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage,image_callback)

    picTime = time.time()
    while (time.time()-picTime < 0.5):
        pass

    t = time.time()
    det_flag=True
    # msg=rospy.wait_for_message("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage)
    # image_deal(msg)
    while has_rev == False:
        pass
    sub.unregister()

    det_flag=False
    print "------------------------"
    print "Waiting for picture to transform, used: ", int((time.time()-t)*1000), "ms"
    print "------------------------"
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

    t = time.time()
    sub=rospy.Subscriber("/tb"+tb_name+"/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage,image_callback_top)
    picTime = time.time()
    while (time.time()-picTime < 0.5):
        pass
    print "============ TIME 1: ", int((time.time()-t)*1000)
    det_flag_top=True
    # msg=rospy.wait_for_message("/tb"+tb_name+"/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage)
    print "============ TIME 2: ", int((time.time()-t)*1000)
    # image_deal_top(msg)
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




start_time = time.time()

def image_callback(msg):
    global net, meta, cam_param, det_flag, has_rev, netInitFinished,np_arr,start_time

    if netInitFinished == False:
        model_name = 'all_640_xy'
        steps = '170000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="./cam_param/"+tb_name+"_bottom.yaml"
        f = open(filename)
        cam_param = yaml.load(f)
        netInitFinished = True
    '''
    print ""
    print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    print time.time()-start_time
    print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    print ""
    print "------------------------"
    '''
    t = time.time()
    if det_flag:
        det_flag=False
        np_arr = np.fromstring(msg.data, np.uint8)
        has_rev = True
    # print "Time used int imgaeCallback: ", int((time.time()-t)*1000), "ms"
    # print "------------------------"

def image_deal(msg):
    global net, meta, cam_param, det_flag, has_rev, netInitFinished,np_arr,start_time

    if netInitFinished == False:
        model_name = 'all_640_xy'
        steps = '170000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="./cam_param/"+tb_name+"_bottom.yaml"
        f = open(filename)
        cam_param = yaml.load(f)
        netInitFinished = True
    '''
    print ""
    print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    print time.time()-start_time
    print "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
    print ""
    print "------------------------"
    '''
    t = time.time()
    if det_flag:
        det_flag=False
        np_arr = np.fromstring(msg.data, np.uint8)
        has_rev = True
    # print "Time used int imgaeCallback: ", int((time.time()-t)*1000), "ms"
    # print "------------------------"


def image_callback_top(msg):
    global  net_top, meta_top, cam_param_top, det_flag_top, has_rev_top, netInitFinished_top,np_arr_top 
    if netInitFinished_top == False:
        model_name = 'door'
        steps = '330000'
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

def image_deal_top(msg):
    global  net_top, meta_top, cam_param_top, det_flag_top, has_rev_top, netInitFinished_top,np_arr_top 
    if netInitFinished_top == False:
        model_name = 'door'
        steps = '330000'
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


#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
# point_control2.init()
# velocity_control.init()
motion_control.init()
# point_control2.reset_odom()
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage, image_callback_top)
rospy.Subscriber("isSlaverFindBall",std_msgs.msg.Bool,slaver_callback)
pub_master = rospy.Publisher("masterInstruction",std_msgs.msg.String,queue_size=1)
# init picture.
getPicture()
getPicture_top()
# ---------------------------------init--------------------------------------------


state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'


anykey = raw_input("============= Press any key to start =========== \n")
print "================  a key is pressed. Go go go! ========================="


carState = state_begin       # begin with "findBall"
while not rospy.is_shutdown():

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        motion_control.PointMove(0.8, -0.5, 0)      # begin to move to a point.
        carState = state_findBall

    elif carState == state_findBall:
        print "---------------------  <state> find ball ---------------------"
        motion_control.rotateTo2(0)        # to face the front
        img = getPicture()
        if isBallNear(img):
            motion_control.PointMove(0.3, 0, 0)
            continue

        print "The ball is NOT near -- by HSV. Use yolo now."
        if len(result['football']) > 1:
            masterFindBall = True
            ballX, ballY = result['football']
            print "Find football at: ", ballX, ballY
            # anykey = raw_input("============= Press any key to Move to ball... =========== \n")
            if abs(ballY) < 0.1:            # the ball is in the front
                print "Robot is facing the ball. Move forward."
                motion_control.PointMove(ballX + 0.4, 0, 0)
            else:
                print "Robot is NOT facing the ball, try to go to the behind."
                motion_control.PointMove(ballX - 0.2, ballY, 0)

        else:
            print "No ball found by yolo, ",
            if isCarNear(img):
                print "a car is NEAR. Do not move back"
                continue
            else:
                print "a car is NOT near. Change to moveback"
                carState = state_changePosition

    elif carState == state_changePosition:
        print "---------------------  <state> change position ---------------------"
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

            if doorX < 1.5:
                print "<BaseLine> The door is NOT far away, can move back."
                motion_control.rotateTo2(0)
                motion_control.PointMove(-0.2, 0, 0)
            elif doorX > 1.8:
                print "<BaseLine> Almost myhalf middle line. Move forward a little"
                motion_control.rotateTo2(0)
                motion_control.PointMove(0.2, 0, 0)
            else:
                print "<BaskLine> Just in position, do not move back or forward. Find ball again."
        # always change to "findball" state after changing position
        carState = state_findBall

    else:
        print "==> [Error]: error state"
        anykey = raw_input("Error at state changing.\n")







