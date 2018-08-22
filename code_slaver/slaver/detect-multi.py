import sys, os
sys.path.append(os.path.join(os.getcwd(),'darknet/python/'))

import darknet as dn
import cv2
import matplotlib.pyplot as plt
import time
from location import allposition
# Darknet
model_name = 'all_640_xy'
steps = '160000'
base_dir = '/home/momenta/mc3/python/weight/weights-'+model_name
# im_dir = '/home/momenta/mc3/python/test_pic/pictures-20180727/'
# im_dir = '/home/momenta/mc3/python/test_pic/awb_off/'
# im_dir = '/home/momenta/mc3/python/test_pic/pic/'
# im_dir = '/home/momenta/mc3/python/test_pic/test_pic_1/'
im_dir = '/home/momenta/mc3/python/rubishbin'
out_base_dir = os.path.join(im_dir,model_name)
if not os.path.exists(out_base_dir):
    os.makedirs(out_base_dir)
out_dir = os.path.join(out_base_dir,model_name+'-'+steps)
if not os.path.exists(out_dir):
    os.makedirs(out_dir)
output_file = os.path.join(out_dir,"result.txt")
wf = open(output_file,'w')

net = dn.load_net(os.path.join(base_dir,"yolov3-tiny-"+model_name+".cfg"), 
	os.path.join(base_dir,"yolov3-tiny-"+model_name+"_"+steps+".weights"), 0)
meta = dn.load_meta(os.path.join(base_dir,"voc-"+model_name+".data"))

import yaml
filename="cam_top.yaml"
f = open(filename)
cam_param = yaml.load(f)

file_names = os.listdir(im_dir)
file_names.sort()
res = []
for i,name in enumerate(file_names):
    if 'jpg' not in name:
        continue
    wf.write("\n{}\n".format(name))
    filename = os.path.join(im_dir,name)
    start_t = time.time()
    r = dn.detect(net, meta, filename,thresh=.3, hier_thresh=.5, nms=0.45)
    re = allposition(r,cam_param)
    re['filename'] = name
    res.append(re)
    end_t = time.time()
    t_cost = round(end_t-start_t,3)
    im = cv2.imread(filename)
    for i in r:
      # print i
     	wf.write("{},{}\n".format(i[0],i[1]))
        sx1,sy1,sx2,sy2=i[2]
        sx1=sx1-sx2/2
        sy1=sy1-sy2/2
        cv2.rectangle(im,(int(sx1),int(sy1)),(int(sx1+sx2),int(sy1+sy2)),(0,255,0),3)
        if (sy1 > 10):
            cv2.putText(im, i[0]+"-"+str(round(i[1],2)), (int(sx1),int(sy1-6)), cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
        else:
            cv2.putText(im, i[0]+"-"+str(round(i[1],2)), (int(sx1),int(sy1+15)), cv2.FONT_HERSHEY_COMPLEX_SMALL,0.8, (0, 255, 0) )
    out_path = os.path.join(out_dir,name+str(t_cost)+'.jpg')
    cv2.imwrite(out_path,im)

import json
output_json = os.path.join(out_dir,"location.json")
f = open(output_json,'w')
f.write(json.dumps(res,indent=2))