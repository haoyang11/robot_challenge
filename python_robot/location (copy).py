import os
import cv2
import json
import numpy as np
import yaml
np.set_printoptions(suppress=True)


def read_cam_param(cam_param):
	K = np.array(cam_param["camera_matrix"]["data"]).reshape((3, 3))
	dist_coeff = np.array(cam_param["distortion_coefficients"]["data"])
	P=np.array(cam_param["projection_matrix"]["data"]).reshape((3,4))
	T = np.array(cam_param["camera_height"])
	return K, dist_coeff, P, T

def bird_view_proj(u,v,cam_param):
	# read param
	K, dist_coeff, P_new, T = read_cam_param(cam_param)
	fx = K[0, 0]																																																																																																																																																																																																															
	fy = K[1, 1]
	cx = K[0, 2]
	cy = K[1, 2]
	fx = P_new[0, 0]																																																																																																																																																																																																															
	fy = P_new[1, 1]
	cx = P_new[0, 2]
	cy = P_new[1, 2]
	pt = np.array([u, v]).reshape((1, 1, 2))
	undist_pt=cv2.undistortPoints(pt,K,dist_coeff,P=P_new)
	undist_pt = undist_pt[0][0]
	undist_pt=[u,v]
	bv_x = fy / (undist_pt[1] - cy) * T
	bv_x = fy / (undist_pt[1] - cy) * T if v > cy + 1 else 10
	bv_y = - (undist_pt[0] - cx) / fx * bv_x																																																																																																			
	return bv_x, bv_y


def  fitfun(x,a,b):
  return a/x+b


def ballpostion(balldata,cam_param):
	# balldata=('ball', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531))
	#parameter
	pic_h=480
	pic_w=640
	line_h=[8179.40058143,3.98672284]
	line_w=[8889.10149816,-0.0269732 ]
	ratio=1
	thresd=0.2
	border=5  #pixel
	coord=balldata[-1]
	[u,v]=[coord[0],coord[1]+coord[3]/2]  # center of bottom line
	[w,h]=[coord[2],coord[3]]
	try:
		ballx,bally = bird_view_proj(u*1.0, v, cam_param)
		x_w=fitfun(w,line_w[0],line_w[1])/100
		x_h=fitfun(h,line_h[0],line_h[1])/100
		# select
		ball_ratio=w/h
		# print("ballratio:",ball_ratio)
		if abs(ball_ratio-ratio)<thresd:
			# print "ratio:",x_w
			ballx=x_w
		if (u-w/2-border)<0 or (u+w/2+border) > pic_w:
			# print "x_h:",x_h
			ballx=x_h

		if (v-h/2-border)<0 or (v+h/2+border) > pic_h:
			# print "x_w:",x_w
			ballx=x_w
	except:
		ballx,bally=[0,0]
	if ballx>3:
		ballx=3
	if abs(bally)>1:
		bally=1

	# part of ball)
	# print("ball postion:x="+str(ballx)+",y="+str(bally))
	return ballx,bally

def doorpostion(doordata,cam_param):
	# balldata=('door', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531))
	#parameter
	pic_h=480
	pic_w=640
	coord=doordata[-1]
	[u,v]=[coord[0],coord[1]+coord[3]/2]
	[w,h]=[coord[2],coord[3]]
	try:
		doorx,doory = bird_view_proj(u*1.0, v, cam_param)
	except:
		doorx,doory=[0,0]
	if doorx>3:
		doorx=3
	if abs(doory)>1:
		doory=1
	# print("door postion:x="+str(doorx)+",y="+str(doory))
	return doorx,doory

def robotpostion(robotdata,cam_param):
	# balldata=('door', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531))
	#parameter
	pic_h=480
	pic_w=640
	coord=robotdata[-1]
	[u,v]=[coord[0],coord[1]+coord[3]/2]
	[w,h]=[coord[2],coord[3]]
	# print robotdata
	try:
		robotx,roboty = bird_view_proj(u*1.0, v, cam_param)
	except:
		robotx,roboty=[0,0]
	if robotx>3:
		 robotx=3
	if abs(roboty)>1:
		roboty=1
	# print("robot postion:x="+str(robotx)+",y="+str(roboty))
	return robotx,roboty


def allposition(data_list,cam_param):
	# data_list=[('door', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531)),
	# 		('robot', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531)),
	# 		('door', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531)),
	# 		('ball', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531))]

	pos_dict={"door":[],"football":[],"robot":[],"conf":[]}

	if len(data_list)<1:
		return pos_dict
	conf=[0,0,0]
	for det in data_list:
		if det[0]=="door":
			conf[0]=det[1]
			pos_dict["door"].append(doorpostion(det,cam_param))
			# print(det[0]+" is ok")
		if det[0]=="robot" and det[1]>conf[1]:
			# print(det[0]+" is ok")
			pos_dict["robot"]=robotpostion(det,cam_param)
			conf[1]=det[1]
		if det[0]=="football" and  det[1]> conf[2]:
			# print(det[0]+" is ok")
			pos_dict["football"]=ballpostion(det,cam_param)
			conf[2]=det[1]

	pos_dict["conf"]=conf
	return  pos_dict


if __name__ == "__main__":
	filename="top.yaml"
	with open(filename) as f:
		cam_param = yaml.load(f)
	balldata=[]
	# ballpostion(balldata,cam_param)
	pos_dict=allposition(balldata,cam_param)
	print(pos_dict)
