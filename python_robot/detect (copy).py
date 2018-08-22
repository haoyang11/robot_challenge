# Stupid python path shit.
# Instead just add darknet.py to somewhere in your python path
# OK actually that might not be a great idea, idk, work in progress
# Use at your own risk. or don't, i don't care

import cv2
import numpy as np

import sys, os
sys.path.append(os.path.join(os.getcwd(),'./darknet/python/'))

import darknet as dn
from location import allposition
import yaml
import time

def detect(net,meta,cam_param,im,isshow=True,issave=False,thresh=.3, hier_thresh=.5, nms=.45, name=None):

    r = dn.detect_np(net,meta,im)
    bdict = {}
    bdict = allposition(r,cam_param)
    if isshow or issave :
        for i in r:
      # print i
            sx1,sy1,sx2,sy2=i[2]
            sx1=sx1-sx2/2
            sy1=sy1-sy2/2
            cv2.rectangle(im,(int(sx1),int(sy1)),(int(sx1+sx2),int(sy1+sy2)),(0,255,0),3)
            if (sy1 > 10):
                cv2.putText(im, i[0]+"-"+str(round(i[1],2)), (int(sx1),int(sy1-6)), cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
            else:
                cv2.putText(im, i[0]+"-"+str(round(i[1],2)), (int(sx1),int(sy1+15)), cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
    if isshow:
        cv2.imshow("pre",im)
        cv2.waitKey(1)
    if issave:
        if name is None:
            name = str(time.time())
        cv2.imwrite("pre-"+name+".jpg",im)
    return bdict

if __name__=="__main__":
    filename="cam_top.yaml"
    f = open(filename)
    cam_param = yaml.load(f)

    # Darknet
    model_name = 'all_640'
    steps = '200000'
    base_dir = '/media/momenta/xy/python/weight/weights-'+model_name
    # im_dir = '/media/momenta/xy/python/test_pic/pictures-20180727/'
    net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"), 
	               os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
    meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))

    im_path = "test100.jpg"

    # r = dn.detect(net, meta, im_path)

    # OpenCV
    im = cv2.imread(im_path)
    r = detect(net, meta, im,isshow=True,issave=True,name=im_path)