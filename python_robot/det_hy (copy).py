
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

import point_control
import velocity_control

t=0
move_start=0
ssr_stage=0
base_dir = "/home/momenta/mc3/python/weight/weights-all/"
net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-all.cfg"), 
                os.path.join(base_dir,"yolov3-tiny-all_80000.weights"), 0)
meta = dn.load_meta(os.path.join(base_dir,"voc-all.data"))

filename="cam_top.yaml"
f = open(filename)
cam_param = yaml.load(f)
# blist=[]
bdict = {}

det_flag=False
has_rev=False
isClose = False
def detecttion(image,flag=True,hsvonly=False,save=True):
    cv2.imshow("imshow",image)
    print"flag,hsvonly,save:",flag,hsvonly,save
    global det_flag
    flag=det_flag
    print "detecttion flag:",flag
    if not flag:  # not detection:flag=false
        print "return none"
        return None
    print "detection"
    # det_flag = False
    global net
    global meta
    global cam_param
    start_t = time.time()
    im = cv2.imread(image)
    ballCenter, isClose = doSomething(im)
    # cv2.imshow("test",im)
    print "doSomething result",ballCenter,isClose
    if hsvonly == True:
        return ballCenter,isClose

    if isClose:
        print "isClose:",isClose
        return ballCenter,isClose
    r = dn.detect(net, meta, image,thresh=0.1)
    end_t = time.time()
    cost_t = round(end_t-start_t)

    # blist = []
    # for i in r:
    #     if i[0] == 'football':
    #         blist = ballpostion(i,cam_param)
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
            # cv2.imwrite("det/"+image+"_pre_"+str(cost_t),im)
    cv2.imshow(str([sx1,sy1,sx2,sy2]),im)
    cv2.waitKey(0)

    #cv2.waitKey(5000)
    #print r
    if len(bdict)<1:
        return None
    # return blist,False
    return bdict,False

def image_callback(msg):
    # print("Received an image!")
    global t
    # global blist
    global bdict,isClose
    global has_rev
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    filename = 'ori'+str(t)+'.jpg'
    print filename
    # cv2.imwrite(filename,image_np)
    flag=True
    bdict,isClose = detecttion(filename,flag=True,hsvonly=True,save=True) 
    flag= False
    bdict,isClose = detecttion(filename,flag=True,save=True) 
    has_rev = True

rospy.init_node('mc3_main', anonymous=True)

rate = rospy.Rate(5)
bridge = CvBridge()
point_control.init()
velocity_control.init()
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
print 'hello'
goal_x, goal_y=[0,0]

while not rospy.is_shutdown():
    print "ssr_stage=",ssr_stage
    det_flag=True
    print "bdict=",bdict
    rate.sleep()