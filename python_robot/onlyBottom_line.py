'''
Only bottom camera.
Detect door and move back.




2018.8.10 1:59

1. Added follow the line movement
2. Master. Begin left-cornor
3. Master baseline distance: 1.7-1.9
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

net_door, net_ball, meta_door, meta_ball = None, None, None, None

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
output_dir_ball = os.path.join(output_dir_base,'ball')
if not os.path.exists(output_dir_ball):
    os.makedirs(output_dir_ball)
output_dir_door = os.path.join(output_dir_base,'door')
if not os.path.exists(output_dir_door):
    os.makedirs(output_dir_door)


# get bottom camera's picture
def getPicture():
    global has_rev, det_flag, np_arr
    has_rev = False

    picTime = time.time()
    while (time.time()-picTime < 1):
        pass
    det_flag=True
    while has_rev == False:
        pass
    det_flag=False
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return image_np

# use bottom camera's paramter to find element
def findElement(img, element):
    global net_door, net_ball, meta_door, meta_ball, cam_param, output_dir_door,output_dir_ball
    img_np = img
    t = time.time()-start_time
    if element == 'door':
        result_dict = detect(net_door,meta_door,cam_param,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir_door)
    elif element == 'ball':
        result_dict = detect(net_ball,meta_ball,cam_param,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir_ball)
    return result_dict


start_time = time.time()
def image_callback(msg):
    global net_door, net_ball, meta_door, meta_ball, cam_param, det_flag, has_rev, netInitFinished,np_arr,start_time
    if netInitFinished == False:
        model_name = 'all_640_xy'
        steps = '170000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net_ball = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta_ball = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        

        model_name = 'door'
        steps = '250000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net_door = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta_door = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))

        filename="./cam_param/"+tb_name+"_bottom.yaml"
        f = open(filename)
        cam_param = yaml.load(f)
        netInitFinished = True

    t = time.time()
    if det_flag:
        det_flag=False
        np_arr = np.fromstring(msg.data, np.uint8)
        has_rev = True


isSlaverAttacking = False
masterFindBall = False
lastAttacker = 'master'

def slaver_callback(msg):
    global masterFindBall, lastAttacker, pub_master
    if msg.data == True:
        print "-------- slaver find ball. ----------"
        if masterFindBall == False:
            isSlaverAttacking = True
            print "M: no.  S: yes.  Slaver attack"
            pub_master.publish('attack')
            lastAttacker = 'slaver'
        else:       # master also find ball.
            if lastAttacker == 'master':
                isSlaverAttacking = False
                print "M: yes. S: yes. Last:M. Slaver standby"
                pub_master.publish('standby')
                
            else:
                isSlaverAttacking = True
                print "M: yes. S: yes. Last:S. Slaver attack. Master standby"
                pub_master.publish('attack')
                
    else:
        isSlaverAttacking = False

#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
# point_control2.init()
# velocity_control.init()
motion_control.init()
# point_control2.reset_odom()
rospy.Subscriber("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
rospy.Subscriber("isSlaverFindBall",std_msgs.msg.Bool,slaver_callback)
pub_master = rospy.Publisher("masterInstruction",std_msgs.msg.String,queue_size=1)

# ---------------------------------init--------------------------------------------


state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'


anykey = raw_input("============= Press any key to start =========== \n")
print "================  a key is pressed. Go go go! ========================="

isNearBaseLine = True
navigateCount = 0

carState = state_begin       # begin with "findBall"
while not rospy.is_shutdown():
    global masterFindBall, isSlaverAttacking, isNearBaseLine,navigateCount

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        motion_control.PointMove(0.8, -0.55, 0)      # begin to move to a point.
        carState = state_findBall

    else:
        print "Find ball..."
        motion_control.rotateTo2(0)
        img = getPicture()

        if carState == state_findBall:
            print "---------------------  <state> find ball ---------------------"
            if isBallNear(img):
                print "mask is near ball but slaver is coming......"
                masterFindBall = True
                isNearBaseLine = False
                motion_control.PointMove(0.4, 0, 0)
                continue

            print "The ball is NOT near -- by HSV. Use yolo now."
            result = findElement(img, 'ball')
            if len(result['football']) > 1:
                masterFindBall = True
                ballX, ballY = result['football']
                if ballX > 0.2:
                    print "Move froward a little. NOT near baseline now."
                    isNearBaseLine = False
                print "Find football at: ", ballX, ballY
                # anykey = raw_input("============= Press any key to Move to ball... =========== \n")
                if isSlaverAttacking == True:
                    print "Slaving is attacking. [MASTER] STANDBY"
                    continue
                else:
                    if abs(ballY) < 0.1:            # the ball is in the front
                        print "Robot is facing the ball. Move forward."
                        motion_control.PointMove(ballX + 0.4, 0, 0)
                    else:
                        print "Robot is NOT facing the ball, try to go to the behind."
                        if ballY > 0:
                            ballY += 0.05
                        else:
                            ballY -= 0.05
                        motion_control.PointMove(ballX - 0.2, ballY, 0)

            else:
                print "No ball found by yolo, ",
                masterFindBall = False
                if isCarNear(img):
                    print "a car is NEAR. Do not move back"
                    continue
                else:
                    print "a car is NOT near. Change to moveback"
                    carState = state_changePosition

        if carState == state_changePosition:
            print "---------------------  <state> change position ---------------------"
            # motion_control.rotateTo2(0)
            result = findElement(img, 'door') # find door info

            if len(result['door']) == 0:
                print "<DOOR> Cannot find door. Move back safely..."
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

                if doorX < 1.7:
                    if isNearBaseLine == True:
                        print "No move back because of near BaseLine"
                    else:
                        print "<BaseLine> The door is NOT far away, can move back."
                        motion_control.rotateTo2(0)
                        motion_control.PointMove(-0.2, 0, 0)
                elif doorX > 1.9:
                    print "<BaseLine> Almost myhalf middle line. Move forward a little"
                    motion_control.rotateTo2(0)
                    motion_control.PointMove(0.2, 0, 0)
                else:
                    print "<BaskLine> Just in position, do not move back or forward. Find ball again."

                if doorX > 1.7:
                    isNearBaseLine = True
                if isNearBaseLine == True:
                    print "Move around the line ..., Times: ", navigateCount
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
                        navigateCount = 0
                    else:
                        print "Error"
                        key = raw_input("Error---\n")
                    navigateCount += 1
                else:
                    navigateCount = 0
            # always change to "findball" state after changing position
            carState = state_findBall









