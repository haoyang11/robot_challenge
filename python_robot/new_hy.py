
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs
import numpy as np
import matplotlib.pyplot as plt
# from yolo import MC3YOLO
from PIL import Image
import time
import os
import sys
sys.path.append(os.path.join(os.getcwd(),'darknet/python/'))
import darknet as dn

from location import allposition
import yaml

from findBall_release import doSomething

import hy
import velocity_control


from  checkBallNear_release import *

from geometry_msgs.msg import Twist, Point # added by dongyan


point_control=hy


t=0
move_start=0
ssr_stage=0
# base_dir = "/home/momenta/mc3/python/weight/weights-all/"
# net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-all.cfg"), 
#                 os.path.join(base_dir,"yolov3-tiny-all_150000.weights"), 0)
# meta = dn.load_meta(os.path.join(base_dir,"voc-all.data"))

model_name = 'all_640'
steps = '170000'
base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"), 
    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))

filename="cam_top.yaml"
f = open(filename)
cam_param = yaml.load(f)
# blist=[]
bdict = {}

det_flag=False
has_rev=False
isClose = False
ballflag = False
def detecttion(image,flag=True,hsvonly=False,save=False):
    global ballflag
    im = cv2.imread(image)
    ballflag=isBallNear(im)
    flag=det_flag

    if not flag:
        return None
    global net
    global meta
    global cam_param
    start_t = time.time()
    im = cv2.imread(image)
    r = dn.detect(net, meta, image,thresh=0.4)
    print r
    end_t = time.time()
    cost_t = round(end_t-start_t)

    bdict = {}
    bdict = allposition(r,cam_param)

    if save :
        for i in r:
        #print i
            sx1,sy1,sx2,sy2=i[2]
            sx1=sx1-sx2/2
            sy1=sy1-sy2/2
            cv2.rectangle(im,(int(sx1),int(sy1)),(int(sx1+sx2),int(sy1+sy2)),(0,255,0),3)
            if (sy1 > 10):
                cv2.putText(im, i[0], (int(sx1),int(sy1-6)), 
                    cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
            else:
                cv2.putText(im, i[0], (int(sx1),int(sy1+15)), 
                    cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
            # cv2.imwrite(image+"_pre_"+str(cost_t),im)
            # cv2.imshow(filename,im)

    #cv2.waitKey(5000)
    #print r
    return bdict

def image_callback(msg):
    print("Received an image!")
    global t
    global bdict,isClose
    global has_rev
    global det_flag
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    filename = 'ori'+str(t)+'.jpg'
    cv2.imwrite(filename,image_np)
    bdict= detecttion(filename,hsvonly=True,save=False) 
    has_rev = True
def back(distance):
    (position_start, rotation) = point_control.get_odom()
    (position, rotation) = point_control.get_odom()
    while (position_start.x-position.x)<distance:
        velocity_control.vel_control(-0.5,0.0)
        (position, rotation) = point_control.get_odom()




rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
point_control.init()
velocity_control.init()
point_control.reset_odom()
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
velocity_control.vel_control(0.0,0.0)
print 'hello'
goal_x, goal_y=[0,0]


base_x=0
base_y=0
base_z=0
state3_flag=True
move2flag=False
while not rospy.is_shutdown():
    # print "ssr_stage=",ssr_stage
    ssr_stage=0
    if ssr_stage==0:
        distance= raw_input("distance\n")
        back(distance)
        # if point_control.PointMove(1.15, 0.4, 0)==4:
        #     print ssr_stage 
        #     print "ssr_stage 0 complete"
        #     ssr_stage=2     
    if ssr_stage==1:#Stage1:find and catch ball
        print "ssr_stage:",ssr_stage
        if move_start==0: # detection

            det_flag=True
            while not has_rev:
                print "wait"
                print "not receive det_flag:",det_flag
                print "ssr_stage:",ssr_stage
                pass 
            if bdict["football"] is None or len(bdict["football"])<1:  # if  not detection  return [] 
                # cv2.waitKey(1000) 
                # velocity_control.vel_control(0,0.5)  #begin rotate until find ball
                
                # print "rotation:"
                print bdict
                (position, rotation) = point_control.get_odom()
                # if point_control.PointMove(position.x-thresld, position.y+goal_y, goal_z)==4:
                # move_start=3
                print "move_start:",move_start
                print (position, rotation) 
                move_start=3
            else:
                print "bdict=",bdict
                goal_x, goal_y = bdict["football"] #run to the ball
                goal_z=0 # rotate to 0 degree
                move_start=1 #finish detection,begin to run to the ball


        if move_start==1:# run to the ball
            has_rev=False
            print "xyz",goal_x, goal_y, goal_z
            print "run to the ball:"
            (position, rotation) = point_control.get_odom()
            print (position, rotation) 
            if point_control.PointMove(position.x+goal_x, position.y+goal_y, goal_z)==4:#finsh running
                move_start=0
            move2flag=True
            if ballflag:
                ssr_stage=2
        

        if move_start==3:  # go back and turn to 1 to find ball
            if not move2flag:
                move_start=0
                print "first finished"
                continue
            thresld=0.15  # back distance
            back(thresld)
            move_start=0
            # if state3_flag:
            #     (position, rotation) = point_control.get_odom()
            #     new_goalx=position.x-thresld
            #     new_goaly=position.y
            #     state3_flag=False
            # if point_control.PointMove(new_goalx, new_goaly,0)==4:#finsh running
            #     move_start=0
            #     state3_flag=True

        print "move_start:",move_start

    if ssr_stage==2:#Stage2:push ball

        print "ssr_stage:",ssr_stage
        print "ballflag:",ballflag
        # goal_x=goal_x+0.4
        velocity_control.vel_control(2,0.0)
        cv2.waitKey(1000)
        if not ballflag:
            ssr_stage=1
        (position, rotation) = point_control.get_odom()
        print "push the ball:"
        print (position, rotation) 
        print position,rotation
        if position.x>=2.3:#have reach the door
            ssr_stage=3

    if ssr_stage==3:#have reach the door
        print "GOAL!!!!"
        break

    rate.sleep() #10hz