'''
Debug version.
To see what's wrong with communication.

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


# def image_callback(msg):
#     global det_flag, has_rev, cv_image
#     if det_flag:
#         det_flag=False
#        # np_arr = np.fromstring(msg.data, np.uint8)  # new deleted
#         cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")      # new added
#         has_rev = True


# def image_callback_top(msg):
#     global  det_flag_top, has_rev_top, cv_image_top
#     if det_flag_top:
#         det_flag_top=False
#        # np_arr = np.fromstring(msg.data, np.uint8)
#         cv_image_top = bridge.imgmsg_to_cv2(msg, "bgr8")
#         has_rev_top = True
#         # print np_arr


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
        pub_master.publish('moveBack')
        if masterFindBall:
            masterState = 'attack'
        else:
            masterState = 'moveBack'
        slaverState = 'moveBack'
        isSlaverAttacking = False


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
rospy.Subscriber("isSlaverFindBall",std_msgs.msg.Bool,slaver_callback)
pub_master = rospy.Publisher("masterInstruction",std_msgs.msg.String,queue_size=1)

# init picture.
camera_init()



statePub = rospy.Publisher("stateOfTwo", std_msgs.msg.String, queue_size=1)


# ---------------------------------init--------------------------------------------

state_begin = 'state_begin'
state_findBall = 'state_findBall'
state_changePosition = 'state_changePosition'


anykey = raw_input("============= Press any key to start =========== \n")
print "================  a key is pressed. Go go go! ========================="


carState = state_begin       # begin with "findBall"
isNearBaseLine = False

g_count = 0

while not rospy.is_shutdown():


    # anykey = raw_input("============= Press any key to continue =========== \n")
    

    global masterFindBall, isSlaverAttacking, isNearBaseLine,navigateCount, masterState, slaverState
    g_count+=1
    statePub.publish(str(g_count)+' - M:' +  masterState + ', S:' + slaverState + ', last: ' + lastAttacker + ', isSlaverAttacking: ' + str(isSlaverAttacking))

    # print "Master find ball? ", masterFindBall

    if carState == state_begin:
        print "---------------------  <state> begin the battle ---------------------"
        carState = state_findBall

    else:
        print "Find ball..."
        img = getPicture()

        if carState == state_findBall:
            # print "---------------------  <state> find ball ---------------------"
            if isBallNear(img):
                # print "Master is near ball but slaver is coming......"
                masterFindBall = True
                # isNearBaseLine = False
                continue

            # print "The ball is NOT near -- by HSV. Use yolo now."
            result = findElement(img)
            if len(result['football']) > 1:
                masterFindBall = True
                # ballX, ballY = result['football']

                # print "Find football at: ", ballX, ballY
                # anykey = raw_input("============= Press any key to Move to ball... =========== \n")
                if isSlaverAttacking == True:
                    print "Slaver is attacking. [MASTER] STANDBY"
                    continue
                else:
                    print "Slaver is NOT attacking. [MASTER] ATTACK !"

            else:
                masterFindBall = False
