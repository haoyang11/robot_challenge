
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
                os.path.join(base_dir,"yolov3-tiny-all_150000.weights"), 0)
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
    global det_flag
    flag=det_flag
    print flag
    if not flag:
        print "return none"
        return None
    print "detecttion"
    det_flag = False
    global net
    global meta
    global cam_param
    start_t = time.time()
    im = cv2.imread(image)
    ballCenter, isClose = doSomething(im)
    if hsvonly == True:
        return ballCenter,isClose

    if isClose:
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
            # cv2.imwrite(image+"_pre_"+str(cost_t),im)
            cv2.imshow(filename,im)

    #cv2.waitKey(5000)
    #print r
    if len(bdict)<1:
        return None
    # return blist,False
    return bdict,False

def image_callback(msg):
    print("Received an image!")
    global t
    # global blist
    global bdict,isClose
    global has_rev
    # try:
    #     # Convert your ROS Image message to OpenCV2
    #     cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # except CvBridgeError, e:
    #     print(e)
    # else:
    #     # Save your OpenCV2 image as a jpeg 
    #     # cv2.imwrite('camera_image.jpeg', cv2_img)
    #    cv2.imshow("Image", cv2_img)
    # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # cv2.imshow("Image", cv2_img)
    # print 'wewewewe'
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # print type(image_np)
    # cv2.imwrite("Image.jpg", image_np)

    # cv2.imshow("image",image_np)
    filename = 'ori'+str(t)+'.jpg'
    cv2.imwrite(filename,image_np)
    # blist = detecttion(filename,flag=True,save=False)
    if(ssr_stage==2):
        flag=True
        # bdict,isClose = detecttion(filename,flag=True,hsvonly=True,save=False) 
        print detecttion(filename,flag,hsvonly=True,save=False)
        bdict,isClose = detecttion(filename,flag=True,hsvonly=True,save=False) 
    else:
        flag=True
        print detecttion(filename,flag,save=False) 
        bdict,isClose = detecttion(filename,flag=True,save=False) 
    has_rev = True
    # cv2.imshow("result", image_np)
    # cv2.waitKey(1000)

# if __name__ == '__main__':
#     mymove=point_control.GotoPoint()
#     # rospy.init_node('mc3_main', anonymous=True)
#     rate = rospy.Rate(10)
#     bridge = CvBridge()
#     rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
#     print 'hello'
#     while not rospy.is_shutdown():
#         print(point_control.msg)
#         (goal_x, goal_y, goal_z) = mymove.getkey()
#         # mymove.PointMove(0.5,0.1,0)
#         mymove.PointMove(goal_x, goal_y, goal_z)
#         (position, rotation) = mymove.get_odom()
#         print position, rotation
#         rate.sleep()
    # rospy.spin()

# mymove=point_control.GotoPoint()
rospy.init_node('mc3_main', anonymous=True)

rate = rospy.Rate(10)
bridge = CvBridge()
point_control.init()
velocity_control.init()
point_control.reset_odom()
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
print 'hello'
goal_x, goal_y=[0,0]

while not rospy.is_shutdown():
    print "ssr_stage=",ssr_stage
    ssr_stage=2
    if ssr_stage==0:
        if point_control.PointMove(1.15, 0.4, 0)==4:
            print ssr_stage 
            print "ssr_stage 0 complete"
            ssr_stage=2     
    if ssr_stage==1:#find and catch ball
        if move_start==0:
            # print(point_control.msg)
            # (goal_x, goal_y, goal_z) = point_control.getkey()
            # move_start=1
            det_flag=True
            while not has_rev:
                print "wait"
            if bdict["football"] is None or len(bdict["football"])<1:  # if  detection  return []  
                velocity_control.vel_control(0,1)
            else:
                print "bdict=",bdict
                _, goal_x, goal_y = bdict["football"]
                goal_z=0
                move_start=1
        if move_start==1:
            has_rev=False
            print "xyz",goal_x, goal_y, goal_z
            if point_control.PointMove(goal_x, goal_y, goal_z)==4:
                move_start=0
    if ssr_stage==2:#push ball
        det_flag=True
        while not has_rev:
            pass
            # print "stage 2 wait"
        y_offset=(bdict[0]-320)/320
        print "y_offset=",y_offset
        velocity_control.vel_control(0.2,y_offset)
        (position, rotation) = get_odom()
        # if point_control.PointMove(2.3, 0.5, 0)==4:
        #     ssr_stage=3
        if position.x>=2.3:
            ssr_stage=3
        if isClose==False:
            ssr_stage=2
    if ssr_stage==3:
        print "GOAL!!!!"
        break

    # print velocity_control.msg
    # (x,y)=velocity_control.getKey()
    # velocity_control.vel_control(x,y)
    rate.sleep()