
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

start_t = time.time()
g_dict = {"door":[],"football":[],"robot":[],"conf":[]}
net_init = True
net = None
meta = None
cam_param = None

call_count = 0

det_flag=False
has_rev=False



def image_callback(msg):
    global start_t, g_dict, net, meta, cam_param, net_init, call_count,det_flag, has_rev

    if net_init:
        model_name = 'all_640'
        steps = '180000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="cam_top.yaml"
        f = open(filename)
        cam_param = yaml.load(f)
        net_init = False

    # commented by dongyan at 22:30
    # call_count += 1
    # if call_count%5 != 0:
    #     return

    # stopped commented

    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    g_isBallNear = isBallNear(image_np)
    print "g_isBallNear, ", g_isBallNear

    # if the ball is near, always push the ball
    if g_isBallNear:
        return

    t = time.time()-start_t
    if det_flag==True:
        print "det_flag == true, "
        g_dict = detect(net,meta,cam_param,image_np,isshow=True,issave=True,name=str(t))
        has_rev=True
    else:
        g_dict = {"door":[],"football":[],"robot":[],"conf":[]}


# t=0
# move_start=0
# ssr_stage=0
# model_name = 'all_640'
# steps = '200000'
# base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
# net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"), 
#     os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
# meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))

# filename="cam_top.yaml"
# f = open(filename)
# cam_param = yaml.load(f)
# # blist=[]
# bdict = {}
# det_flag=False
# has_rev=False

# isClose = False
# g_dict = {"door":[],"football":[],"robot":[],"conf":[]}



# def detecttion(image,flag=True,save=False):
#     if not flag:
#         return None
#     global net
#     global meta
#     global cam_param
#     start_t = time.time()

#     r = dn.detect(net, meta, image,thresh=0.4)
#     # print r
#     end_t = time.time()
#     cost_t = round(end_t-start_t)

#     # blist = []
#     # for i in r:
#     #     if i[0] == 'football':
#     #         blist = ballpostion(i,cam_param)
#     bdict = {}
#     bdict = allposition(r,cam_param)

#     if save :
#         im = cv2.imread(image)
#         for i in r:
#         #print i
#             sx1,sy1,sx2,sy2=i[2]
#             sx1=sx1-sx2/2
#             sy1=sy1-sy2/2
#             cv2.rectangle(im,(int(sx1),int(sy1)),(int(sx1+sx2),int(sy1+sy2)),(0,255,0),3)
#             if (sy1 > 10):
#                 cv2.putText(im, i[0], (int(sx1),int(sy1-6)), 
#                     cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
#             else:
#                 cv2.putText(im, i[0], (int(sx1),int(sy1+15)), 
#                     cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
#         cv2.imwrite(image+"_pre_"+str(cost_t),im)
#         # cv2.imshow(filename,im)

#     #cv2.waitKey(5000)
#     #print r
#     # if len(bdict)<1:
#     #     return None
#     # return blist,False
#     # return bdict,
#     print bdict
#     return bdict




# def image_callback(msg):
#     global g_isBallNear, g_dict ,start_time
#     global meta
#     global net
#     global cam_param
#     '''
#     global t
#     # global blist
#     global bdict,isClose
#     global has_rev
#     '''
#     global det_flag
#     if det_flag:
#         np_arr = np.fromstring(msg.data, np.uint8)
#         image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         g_isBallNear = isBallNear(image_np)

#         # if the ball is near, always push the ball
#         if g_isBallNear:
#             return

#         # use net to find ball
#         filename = 'ori'+str(t)+ '.jpg'
#         cv2.imwrite(filename,image_np)
#         # blist = detecttion(filename,flag=True,save=False)
#         g_dict = detecttion(filename,save=False)
#         # g_dict = detect(meta,net,cam_param,image_np,isshow=True,issave=False)
#     else:
#         g_isBallNear = False
#         g_dict = {"door":[],"football":[],"robot":[],"conf":[]}


#--------------------------------- init--------------------------------------------
rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
point_control.init()
velocity_control.init()
point_control.reset_odom()
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)

print 'hello'
goal_x, goal_y=[0,0]
# ---------------------------------init--------------------------------------------



# ---------------------- new by Dong yan-----------------------
# car state machine
carState_begin = 'carState_begin'
carState_searchBall = 'carState_searchBall'
carState_chaseBall = 'carState_chaseBall'
carState_pushBall = 'carState_pushBall'
carState_wait = 'carState_wait'

# carState = carState_begin

carState = carState_searchBall
# variables in control
g_targetPointX, g_targetPointY = 0, 0
g_targetSpeedLinear, g_targetSpeedAngular = 0, 0


# cv2.waitKey(10000)

start_time = time.time()


velocity_control.vel_control(0.0,0.0)
point_control.PointMove(0, 0, 0)
anykey = raw_input("Press any key to start\n")

det_time=0

while not rospy.is_shutdown():
    global det_time
    # print "----------------- State: ", carState, " ----------------------"
    global g_targetPointX, g_targetPointY, g_targetSpeedLinear, g_targetSpeedAngular
    global g_isBallNear, g_dict
    global start_time

    # print "<Attime>: ", round(time.time()-start_time), ", State: ", carState

    # state change part
    if carState == carState_begin:

        g_targetPointX, g_targetPointY = 0.2, -0.2
        # g_targetPointX, g_targetPointY = 1.15, -0.5
        (currentPoint, currentRotation) = point_control.get_odom()
        distance_error = math.sqrt(pow((currentPoint.x - g_targetPointX),2) + pow((currentPoint.y - g_targetPointY), 2))
        # if distance_error < 0.05:
        carState = carState_searchBall
        print "[State change]==> from carState_begin to carState_searchBall"

    elif carState == carState_searchBall:
        if g_isBallNear:
            carState = carState_pushBall
            print "[State change]==> from carState_searchBall to carState_pushBall"
        else:
            # print g_dict
            # check the network result
            if len(g_dict['football']) >= 1:
                print " ========================  ballPosition: ", g_dict['football']
                g_targetPointX, g_targetPointY = g_dict['football']
                carState = carState_chaseBall
                print "[State change]==> from carState_searchBall to carState_chaseBall"
            else:   
                carState = carState_searchBall
                # print "[State]==> still carState_searchBall"


    elif carState == carState_chaseBall:
        if g_isBallNear:
            carState = carState_pushBall
            print "[State change]==> from carState_chaseBall to carState_pushBall"
        else:
            # check the network result
            if len(g_dict['football']) >= 1:
                g_targetPointX, g_targetPointY = g_dict['football']
                carState = carState_chaseBall
                # print "[State]==> still carState_chaseBall"
            else:   
                carState = carState_searchBall
                print "[State change]==> from carState_chaseBall to carState_searchBall"

    elif carState == carState_pushBall:
        if g_isBallNear:
            carState = carState_pushBall
            # print "[State]==> still carState_pushBall"
        else:
            if len(g_dict['football']) >= 1:
                g_targetPointX, g_targetPointY = g_dict['football']
                carState = carState_chaseBall
                print "[State change]==> from carState_pushBall to carState_chaseBall"
            else:   
                carState = carState_searchBall
                print "[State change]==> from carState_pushBall to carState_searchBall"


    else:
        print "Waiting..."


    print "transfer  stage:",carState

    # control part
    if carState == carState_begin:
        print "Go to point: ", g_targetPointX, g_targetPointY
        point_control.PointMove(g_targetPointX, g_targetPointY, 0)

    elif carState == carState_searchBall:
        # velocity_control.vel_control(0, 0)
        
        search_start_time=time.time()

        det_flag=True
        print "det_flag:",det_flag
        while time.time()-search_start_time<=5: # wait for one second
            print "waitting==========="
            if has_rev:
                det_flag=False
                break
            pass
        
        # while has_rev!=True:
        #     pass
        det_flag=False

        print "Search once......"
        print "has_rev",has_rev
        if has_rev and len(g_dict["football"])>1:

            print "g_dict:",g_dict
            # has_rev=False
            print "has find ballPosition"
            has_rev=False
            continue
        else:
            point_control.PointMove(-0.1, 0, 0)
        has_rev=False
        # has_rev=False
        
        # print "=== Move backward to find ball."

    elif carState == carState_chaseBall:
        det_flag=False
        print " --------------------   chase Ball at: ", g_targetPointX, g_targetPointY
        start_time = time.time()
        point_control.PointMove(g_targetPointX, g_targetPointY, 0)
        print " --------------------   Stopped at: ", g_targetPointX, g_targetPointY,
        print ", Time used: ", round((time.time()-start_time)*1000), "ms."
        # print "=== End Point movement"
        det_flag=True

    elif carState == carState_pushBall:
        # velocity_control.vel_control(5, 0)
        print "Push ball==="
        point_control.PointMove(0.4, 0, 0)    # move 20cm to push ball
        # cv2.waitKey(1000)

    else:
        print "[Error], unexpected state..."
        # velocity_control.vel_control(0, 0)
