'''

Use two camera.
Zxq's new technology.
Delete rotateTo(), use PointMove(x,y,0) instead.

2018.8.14 10:05

Added upd instead of TCP/IP
8.14 21:01


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
from std_msgs.msg import Empty,UInt8MultiArray
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

def recvpic_onconnect(sock,shape=(480,640,3)):
    i = 0
    sock.settimeout(1)
    im = ""
    while True:
        i += 1
        try:
            print "Try",i
            sock.send("Pic Request")
            print "Pic Request"
            recvlen = int(sock.recv(1024))
            sock.send("Start")
            im = ""
            while len(im) < recvlen:
                data = sock.recv(20480)
                im = im+data
            im_arr = np.fromstring(im,dtype=np.uint8)
            res_im = cv2.imdecode(im_arr,cv2.IMREAD_COLOR)
            return res_im
        except socket.timeout:
            print "timeout"

def recvpic(host,shape=(480,640,3)):
    s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    s.connect(host)
    s.send("Pic Request")
    im = ""      
    while len(im) < shape[0]*shape[1]*shape[2]:
        data = s.recv(480*640*3)
        im = im+data
    im = np.reshape(np.fromstring(im,dtype=np.uint8),shape)
    return im


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
    global net_top, meta_top, cam_param_top, output_dir_top
    img_np = img
    t = time.time()-start_time
    result_dict = detect(net_top,meta_top,cam_param_top,img_np,isshow=False,issave=True,name=str(t),output_dir=output_dir_top)
    return result_dict

callBackCount = 0
def image_callback(msg):
    global det_flag, has_rev, cv_image

    global callBackCount
    if det_flag:
        det_flag=False
       # np_arr = np.fromstring(msg.data, np.uint8)  # new deleted
        # cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")      # new added

        print type(msg.data),len(msg.data)
        im_arr = np.array(msg.data)
        print im_arr.shape
        im_arr = im_arr.reshape((None,1))
        print im_arr.shape
        # cv_image = cv2.imdecode(im_arr,cv2.IMREAD_COLOR)

        # callBackCount += 1
        # cv2.imwrite("b"+str(callBackCount) + ".jpg", cv_image)

        # im_arr = 
        # im_arr = np.transpose(im_arr)
        cv_image = cv2.imdecode(im_arr,cv2.IMREAD_COLOR)
        cv2.imwrite("a"+str(callBackCount) + ".jpg", cv_image)

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


isSlaverAttacking = False
masterFindBall = False
lastAttacker = 'master'

# debug parameter
masterState, slaverState = 'attack', 'standby'

def slaver_callback(msg):
    global masterFindBall, lastAttacker, pub_master, masterState, slaverState, isSlaverAttacking

    # if masterFindBall == True:
    #     masterState = 'findBall'
    # else:
    #     masterState = 'noBall'

    if msg.data == True:
        print "-------- slaver find ball. ----------"
        if masterFindBall == False:
            isSlaverAttacking = True
            # print "M: no.  S: yes.  Slaver attack"
            pub_master.publish('attack')
            lastAttacker = 'slaver'
            
            masterState = 'Whatever'
            slaverState = 'attack'

        else:       # master also find ball.
            if lastAttacker == 'master':
                isSlaverAttacking = False
                # print "M: yes. S: yes. Last:M. Slaver standby"
                pub_master.publish('standby')
                masterState = 'attack'
                slaverState = 'standby'

            else:
                masterState = 'Whatever'
                slaverState = 'attack'
                isSlaverAttacking = True
                # print "M: yes. S: yes. Last:S. Slaver attack. Master standby"
                pub_master.publish('attack')
                
    else:
        print "Slaver is attacking changed to 'FALSE' <==========="
        # pub_master.publish('moveBack')
        if masterFindBall:
            masterState = 'attack'
        else:
            masterState = 'moveBack'
        slaverState = 'moveBack'
        isSlaverAttacking = False



#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)

# new added
rospy.Subscriber("/tb"+tb_name+"/pub_camera",UInt8MultiArray,image_callback)
ask_camera=rospy.Publisher("/tb"+tb_name+"/ask_camera", Empty,queue_size=1)
rospy.Subscriber("/tb"+tb_name+"/pub_camera_top",Image,image_callback_top)
ask_camera_top=rospy.Publisher("/tb"+tb_name+"/ask_camera_top", Empty,queue_size=1)

rate = rospy.Rate(10)
bridge = CvBridge()
motion_control.init()
# point_control2.reset_odom()
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
# rospy.Subscriber("/tb"+tb_name+"/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage, image_callback_top)
rospy.Subscriber("isSlaverFindBall",std_msgs.msg.Bool,slaver_callback)
pub_master = rospy.Publisher("masterInstruction",std_msgs.msg.String,queue_size=1)

# init picture.
camera_init()


statePub = rospy.Publisher("stateOfTwo", std_msgs.msg.String, queue_size=1)


# ---------------------------------init--------------------------------------------

state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'


host = ('192.168.1.63',9999)
sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock.connect(host)
host_top = ('192.168.1.64',9999)
sock_top = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
sock_top.connect(host)

anykey = raw_input("============= Press any key to start =========== \n")
print "================  a key is pressed. Go go go! ========================="


carState = state_findBall       # begin with "findBall"
isNearBaseLine = False
navigateCount = 0
while not rospy.is_shutdown():
    global masterFindBall, isSlaverAttacking, isNearBaseLine,navigateCount

    global lastAttacker

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        motion_control.PointMove(0.8, -0.55, 0)      # begin to move to a point.
        carState = state_findBall

    else:
        t = time.time()
        # print "Find ball..."
        # img = recvpic(host)
        img = recvpic_onconnect(sock)
        # img = getPicture()
        print "<----------------> Time used by bottom camera: ", (int)((time.time()-t)*1000), "ms.   ?????????"

        # print "Time used in 'getPicture': ", (int)((time.time()-t)*1000)
        if carState == state_findBall:
            # print "---------------------  <state> find ball ---------------------"
            if isBallNear(img):
                # print "Master is near ball but slaver is coming......"
                masterFindBall = True
                isNearBaseLine = False


                


                if isSlaverAttacking == True:
                    lastAttacker = 'slaver'
                    continue
                else:
                    lastAttacker = 'master'
                    motion_control.PointMove(0.4, 0, 0)
                    continue

            # print "The ball is NOT near -- by HSV. Use yolo now."
            result = findElement(img)
            if len(result['football']) > 1:
                masterFindBall = True
                ballX, ballY = result['football']
                if ballX > 0.2:
                    # print "Move froward a little. NOT near baseline now."
                    isNearBaseLine = False

                print "Find football at: ", ballX, ballY
                # anykey = raw_input("============= Press any key to Move to ball... =========== \n")
                if isSlaverAttacking == True:
                    # print "Slaving is attacking. [MASTER] STANDBY"
                    lastAttacker = 'slaver'
                    continue
                else:
                    lastAttacker = 'master'
                    if abs(ballY) < 0.1:            # the ball is in the front
                        # print "Robot is facing the ball. Move forward."
                        motion_control.PointMove(ballX + 0.4, 0, 0)
                    else:
                        # print "Robot is NOT facing the ball, try to go to the behind."
                        if ballY > 0:
                            ballY += 0.05
                        else:
                            ballY -= 0.05
                        motion_control.PointMove(ballX - 0.2, ballY, 0)

            else:
                # print "No ball found by yolo, ",
                masterFindBall = False
                if isCarNear(img):
                    # print "a car is NEAR. Do not move back"
                    continue
                else:
                    # print "a car is NOT near. Change to moveback"
                    carState = state_changePosition

        if carState == state_changePosition:
            # print "---------------------  <state> change position ---------------------"
            # motion_control.rotateTo2(0)
            t = time.time()
            # img = getPicture_top()
            img = recvpic_onconnect(sock_top)
            print "<----------------> Time used by topCamera: ", (int)((time.time()-t)*1000), "ms.   ?????????"

            result = findElement_top(img) # find door info
            if len(result['door']) == 0:
                # print "<DOOR> Cannot find door. Move back safely..."
                # motion_control.rotateTo2(0)
                motion_control.PointMove(-0.3, 0, 0)
            else:
                # doorInfo = result['door']
                doorX, doorY = 0, 0
                if len(result['door']) == 1:
                    doorX, doorY = result['door'][0][0], result['door'][0][1]
                    # print "<DOOR> Find one pillar at:", doorX, doorY
                else:
                    d1_x, d1_y = result['door'][0][0], result['door'][0][1]
                    d2_x, d2_y = result['door'][1][0], result['door'][1][1]
                    print "<DOOR> Find two pillars, at:", d1_x, d1_y,", 2: ", d2_x, d2_y
                    # doorX = (d1_x + d2_x) * 0.5
                    doorX = max(d1_x, d2_x)     # use the more near one to check the distance
                    doorY = (d1_y + d2_y) * 0.5 # doorY is not used so far.

                if doorX < 1.7:
                    if isNearBaseLine == True:
                        # print "No move back because of near BaseLine"
                        pass
                    else:
                        # print "<BaseLine> The door is NOT far away, can move back."
                        # motion_control.rotateTo2(0)
                        motion_control.PointMove(-0.15, 0, 0)
                elif doorX > 1.9:
                    # print "<BaseLine> Almost myhalf middle line. Move forward a little"
                    # motion_control.rotateTo2(0)
                    motion_control.PointMove(0.2, 0, 0)
                else:
                    # print "<BaskLine> Just in position, do not move back or forward. Find ball again."
                    pass

                if doorX > 1.7:
                    isNearBaseLine = True

                if isNearBaseLine == True:

                    if doorY > 0:
                        deltaY = doorY + 0.05
                    else:
                        deltaY = doorY - 0.05
                    motion_control.PointMove(0, deltaY, 0)
                    '''
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
                    '''
                else:
                    navigateCount = 0
            # always change to "findball" state after changing position
            carState = state_findBall









