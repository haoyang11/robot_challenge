'''

Use two camera.
Zxq's new technology.
Delete rotateTo(), use PointMove(x,y,0) instead.

2018.8.14 10:05
Slaver

2018.8.15 back to middle of door.
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
from std_msgs.msg import Empty 



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


def getPicture():
    global has_rev, det_flag, np_arr,ask_camera
    has_rev = False
    # picTime = time.time()
    # while (time.time()-picTime < 0.01):
    #     pass
    det_flag=True
    ask_camera.publish(empty_msg)      # new added
    while has_rev == False:
        pass
    det_flag=False
    # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return cv_image

# use bottom camera's paramter to find element
def findElement(img):
    global net, meta, cam_param, output_dir
    img_np = img
    t = time.time()-start_time
    result_dict = detect(net,meta,cam_param,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir)
    return result_dict

# top camera's function.
def getPicture_top():
    global has_rev_top, det_flag_top, np_arr_top,ask_camera_top
    has_rev_top = False

    # picTime = time.time()
    # while (time.time()-picTime < 0.01):
    #     pass
    det_flag_top=True
    ask_camera_top.publish(empty_msg)
    while has_rev_top == False:
        pass
    det_flag_top=False
    # image_np = cv2.imdecode(np_arr_top, cv2.IMREAD_COLOR)
    return cv_image_top

def findElement_top(img):
    global net_top, meta_top, cam_param_top, output_dir
    img_np = img
    t = time.time()-start_time
    result_dict = detect(net_top,meta_top,cam_param_top,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir)
    return result_dict


def image_callback(msg):
    global det_flag, has_rev, cv_image
    if det_flag:
        det_flag=False
       # np_arr = np.fromstring(msg.data, np.uint8)  # new deleted
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")      # new added
        has_rev = True


def image_callback_top(msg):
    global  det_flag_top, has_rev_top, cv_image_top
    if det_flag_top:
        det_flag_top=False
       # np_arr = np.fromstring(msg.data, np.uint8)
        cv_image_top = bridge.imgmsg_to_cv2(msg, "bgr8")
        has_rev_top = True
        # print np_arr


def camera_init():
    global  net_top, meta_top, cam_param_top
    global net, meta, cam_param

    # bottom camera
    model_name = 'all_640'
    steps = '190000'
    base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
    net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
    meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
    filename="cam_param/"+tb_name+"_bottom.yaml"
    f = open(filename)
    cam_param = yaml.load(f)

    # top camera
    model_name = 'all_640'
    steps = '190000'
    base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
    net_top = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
    meta_top = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
    filename="cam_param/"+tb_name+"_top.yaml"
    f = open(filename)
    cam_param_top = yaml.load(f)
    netInitFinished_top = True


slaverFindBall = False
slaverInstruction = 'waitForResponse'

def slaver_publish():
    global slaverFindBall,pub_slaver
    print "Slaver publish a: '", slaverFindBall, "'."

    slaverInstruction = 'waitForResponse'

    if slaverFindBall == True:
        pub_slaver.publish(True)
    else:
        pub_slaver.publish(False)


def master_callback(msg):
    global slaverInstruction

    if msg.data == 'attack':
        slaverInstruction = 'attack'
    elif msg.data == 'standby':
        slaverInstruction = 'standby'
    elif msg.data == 'moveBack':
        slaverInstruction = 'moveBack'
        
    else:
        print "<CALL BACK> [ERROR]."
        print msg.data
        print "======="
        anykey = raw_input("Error.\n")

    print "< Master CALL BACK >. Slaver received ' ", slaverInstruction, " '."





#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)

# new added
rospy.Subscriber("/tb"+tb_name+"/pub_camera",Image,image_callback)
ask_camera=rospy.Publisher("/tb"+tb_name+"/ask_camera", Empty,queue_size=1)
rospy.Subscriber("/tb"+tb_name+"/pub_camera_top",Image,image_callback_top)
ask_camera_top=rospy.Publisher("/tb"+tb_name+"/ask_camera_top", Empty,queue_size=1)

rate = rospy.Rate(10)
bridge = CvBridge()
motion_control.init()
# point_control2.reset_odom()
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage, image_callback_top)
pub_slaver = rospy.Publisher("isSlaverFindBall",std_msgs.msg.Bool,queue_size=1)
rospy.Subscriber('masterInstruction',std_msgs.msg.String,master_callback)

# init picture.
camera_init()


# ---------------------------------init--------------------------------------------

state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'


anykey = raw_input("============= Press any key to start =========== \n")
print "================  a key is pressed. Go go go! ========================="


carState = state_begin       # begin with "findBall"
isNearBaseline = True
navigateCount = 0
haveSeeTwoPillars = False

while not rospy.is_shutdown():

    global slaverFindBall, slaverInstruction, isNearBaseline, navigateCount

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        motion_control.PointMove(0.4, 0.4, 0)      # begin to move to a point.

        # rotate to wait
        motion_control.rotateTo2(0)
        motion_control.rotateTo2(90)
        motion_control.rotateTo2(179)
        motion_control.rotateTo2(-90)
        motion_control.rotateTo2(0)

        carState = state_findBall
    else:
        # motion_control.rotateTo2(0)        # to face the front
        img = getPicture()
        if carState == state_findBall:
            print "---------------------  <state> find ball ---------------------"
            # anykey = raw_input("============= Press any key to continue =========== \n")
            
            if isBallNear(img):
                slaverFindBall = True
                isNearBaseline = False
                slaver_publish()
                while slaverInstruction == 'waitForResponse':
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
                    print slaverInstruction
                    # print "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE"
                    anykey = raw_input("Error.\n")

            print "The ball is NOT near -- by HSV. Use yolo now."
            result = findElement(img)
            if len(result['football']) > 1:

                slaverFindBall = True
                slaver_publish()
                while slaverInstruction == 'waitForResponse':
                    print "Waiting for response --",
                print ""
                print "Received instruction: ", slaverInstruction

                ballX, ballY = result['football']
                if ballX > 0.2:
                    print "Move forward a little. Nor near baseline now."
                    isNearBaseline = False

                print "Find football at: ", ballX, ballY
                # anykey = raw_input("============= Press any key to Move to ball... =========== \n")
                if slaverInstruction == 'attack':
                    print "<----------->    [ SLAVER ]   ATTACK    <--------------->"
                    if abs(ballY) < 0.1:            # the ball is in the front
                        print "Robot is facing the ball. Move forward."
                        motion_control.PointMove(ballX + 0.4, 0, 0)
                    else:
                        print "Robot is NOT facing the ball, try to go to the behind."
                        # ball offset because of location stopped at around 5cm 
                        if ballY > 0:
                            ballY += 0.05
                        else:
                            ballY -= 0.05
                        motion_control.PointMove(ballX - 0.2, ballY, 0)
                elif slaverInstruction == 'standby':
                    print "<----------->    [ SLAVER ]   STANDYBY    <--------------->"
                    continue
                elif slaverInstruction == 'moveBack':
                    print "<----------->    [ SLAVER ]   MOVEBACK    <--------------->"
                    
                else:
                    print "[ERROR] at slaverInstruction"
                    anykey = raw_input("Error.\n")
                # carState remains to be state_findBall

            else:
                slaverFindBall = False
                slaver_publish()
                # print "No ball fond by yolo, ",
                if isCarNear(img):
                    # print "a car is near. Do not move back."
                    continue
                else:
                    # print "no car near. Change to moveBack"
                    carState = state_changePosition

        if carState == state_changePosition:
            
            print "---------------------  <state> change position ---------------------"
            # anykey = raw_input("============= Press any key to continue =========== \n")
            # motion_control.rotateTo2(0)
            img  = getPicture_top()
            result = findElement_top(img)
            if len(result['door']) == 0:
                print "<DOOR> Cannot find door. Move back safely..."
                if isNearBaseline == True:
                    print "NO move back because of near BaseLine"
                else:
                    motion_control.rotateTo2(0)
                    motion_control.PointMove(-0.3, 0, 0)
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
                    haveSeeTwoPillars = True


                if haveSeeTwoPillars:
                    print "See two pillars when moving back. Then move to the middle."
                    if doorY > 0:
                        deltaY = doorY - 0.04
                    else:
                        deltaY = doorY + 0.04
                    motion_control.PointMove(0, deltaY, 0)

                if doorX < 1.9:
                    
                    print "<BaseLine> The door is NOT far away, can move back."
                    
                    if isNearBaseline == True:
                        print "NO move back because of near BaseLine"
                    else:
                        motion_control.rotateTo2(0)
                        motion_control.PointMove(-0.2, 0, 0)

                elif doorX > 2.1:
                    print "<BaseLine> Almost near baseline. Move forward a little"
                    motion_control.rotateTo2(0)
                    motion_control.PointMove(0.2, 0, 0)
                else:
                    print "<BaskLine> Just in position, do not move at all. Find ball again."

                if doorX > 1.9:
                    isNearBaseline = True

                
                if isNearBaseline == True:
                    print "Move around the line..., Times: ", navigateCount
                    if navigateCount == 0:
                        pass
                    elif navigateCount == 1:
                        motion_control.rotateTo2(90)
                        motion_control.PointMove(0.4, 0, 0)
                    elif navigateCount == 2 or navigateCount == 3:
                        motion_control.rotateTo2(-90)
                        motion_control.PointMove(0.4, 0, 0)
                    elif navigateCount == 4:
                        motion_control.rotateTo2(90)
                        motion_control.PointMove(0.4, 0, 0)
                        navigateCount = 0       #reset the count
                    else:
                        print "ERROR!"
                        key = raw_input("Error in count\n")
                    navigateCount += 1
                else:
                    navigateCount = 0
                '''
                if isNearBaseline == True:
                    if len(result['door'])>1:
                        motion_control.PointMove(doorX, doorY, 0)
                '''



            # always change to "findball" state after changing position
            carState = state_findBall

        # else:
        #     print "==> [Error]: error state"

