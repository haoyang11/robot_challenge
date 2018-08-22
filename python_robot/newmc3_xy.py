
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs
import numpy as np

from location import allposition
import yaml

import point_control
import velocity_control

# import by xy
import time
import os
import sys
sys.path.append(os.path.join(os.getcwd(),'darknet/python/'))
import darknet as dn
from detect import detect

start_t = time.time()
top_bdict = {}
top_net_init = True
top_net = None
top_meta = None
top_cam_param = None

top_call_count = 0

bottom_bdict = {}
bottom_net_init = True
bottom_net = None
bottom_meta = None
bottom_cam_param = None

bottom_call_count = 0

base_out_dir = "run_pic/test4"
if not os.path.exists(base_out_dir):
    os.makedirs(base_out_dir)
top_out_dir = os.path.join(base_out_dir,'top')
if not os.path.exists(top_out_dir):
    os.makedirs(top_out_dir)
bottom_out_dir = os.path.join(base_out_dir,'bottom')
if not os.path.exists(bottom_out_dir):
    os.makedirs(bottom_out_dir)
top_json_path = os.path.join(top_out_dir,'location.json')
bottom_json_path = os.path.join(bottom_out_dir,'location.json')
import json
top_wf = open(top_json_path,'w')
bottom_wf = open(bottom_json_path,'w')

def image_callback_top(msg):

    global start_t, top_bdict, top_net, top_meta, top_cam_param, top_net_init, top_call_count
    global top_wf
    print "Top Call back ...", top_call_count
    if top_net_init:
        model_name = 'door'
        steps = '240000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        top_net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        top_meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="cam_top.yaml"
        f = open(filename)
        top_cam_param = yaml.load(f)
        top_net_init = False

    top_call_count += 1
    if top_call_count%5 != 0:
        return
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    t = str(time.time()-start_t)
    top_bdict = detect(top_net,top_meta,top_cam_param,image_np,isshow=False,issave=True,output_dir=top_out_dir,name= t)
    top_info = top_bdict
    top_info['filename'] = t
    top_wf.write(json.dumps(top_info,indent=2))
    top_wf.write("\n")

def image_callback(msg):

    global start_t, bottom_bdict, bottom_net, bottom_meta, bottom_cam_param, bottom_net_init, bottom_call_count
    global bottom_wf
    print "Bootom Call back ...", bottom_call_count
    if bottom_net_init:
        model_name = 'all_640_xy'
        steps = '160000'
        base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
        bottom_net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"),  
                    os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
        bottom_meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))
        filename="cam_top.yaml"
        f = open(filename)
        bottom_cam_param = yaml.load(f)
        bottom_net_init = False

    bottom_call_count += 1
    if bottom_call_count%5 != 0:
        return
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    t = str(time.time()-start_t)
    bottom_bdict = detect(bottom_net,bottom_meta,bottom_cam_param,image_np,isshow=False,issave=True,output_dir=bottom_out_dir,name= t)
    bottom_info = bottom_bdict
    bottom_info['filename'] = t
    bottom_wf.write(json.dumps(bottom_info,indent=2))
    bottom_wf.write("\n")

# mymove=point_control.GotoPoint()
rospy.init_node('mc3_main', anonymous=True)
rate = rospy.Rate(10)
bridge = CvBridge()
# point_control.init()
# velocity_control.init()
rospy.Subscriber("/raspicam_node_top/image/compressed",sensor_msgs.msg.CompressedImage, image_callback_top)
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
print 'hello'
goal_x, goal_y=[0,0]

while not rospy.is_shutdown():
    rate.sleep()