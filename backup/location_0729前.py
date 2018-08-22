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
	# balldata=('door', 0.8986098766326904, (465.4346008300781, 229.2826690673828, 26.574583053588867, 124.28572082519531))
	#parameter
	pic_h=480
	pic_w=640
	line_h=[8179.40058143,3.98672284]
	line_w=[8889.10149816,-0.0269732 ]
	ratio=1
	thresd=0.2
	border=5  #pixel
	coord=balldata[-1]
	[u,v]=[coord[0],coord[1]+coord[3]/2]
	[w,h]=[coord[2],coord[3]]
	try:
		ballx,bally = bird_view_proj(u*1.0, v, cam_param)
		x_w=fitfun(w,line_w[0],line_w[1])/100
		x_h=fitfun(h,line_h[0],line_h[1])/100
		# select
		ball_ratio=w/h
		if abs(ball_ratio-ratio)<thresd:
			ballx=x_w
		if (u-w/2-border)<0 or (u+w/2+border) > pic_w:
			ballx=x_h

		if (v-h/2-border)<0 or (v+h/2+border) > pic_h:
			ballx=x_w
	except:
		ballx,bally=[0,0]

	# part of ball)
	if ballx>2.5:
		ballx=2.5
	if abs(bally)>1:
		bally=1
	print("ball postion:x="+str(ballx)+",y="+str(bally))
	return ["ball",ballx,bally]


if __name__ == "__main__":
	filename="top.yaml"
	with open(filename) as f:
		cam_param = yaml.load(f)
	balldata=[]
	ballpostion(balldata,cam_param)
