# -*- coding: utf-8 -*-

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import Empty
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import os
tb_name=str(os.getenv('TB_NAME'))

def ask_callback(msg):
    global ask
    print "I rev ask"
    ask=True

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size = (640, 480))
# allow the camera to warmup
time.sleep(1)
# capture frames from the camera
rospy.init_node("tb"+tb_name+'camera', anonymous=True)
pub_camera=rospy.Publisher("/tb"+tb_name+"/pub_camera",Image,queue_size=1)
rospy.Subscriber("/tb"+tb_name+"/ask_camera", Empty, ask_callback)
bridge = CvBridge()

ask=False
rate=rospy.Rate(100)

print "ready to send image"

while not rospy.is_shutdown(): 
    if ask==True:
        ask=False
        camera.capture(raw_capture,format="bgr")#,resize=(640, 480)
        cv_image = raw_capture.array
        ros_msg=bridge.cv2_to_imgmsg(cv_image, "bgr8")#cv2_to_compressed_imgmsg
        pub_camera.publish(ros_msg)
        raw_capture.truncate(0)
        print "shoot"
    rate.sleep()

# for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
#     # show the frame
#     cv_image = frame.array
#     ros_msg=bridge.cv2_to_imgmsg(cv_image, "bgr8")#cv2_to_compressed_imgmsg
#     pub_camera.publish(ros_msg)
#     # clear the stream in preparation for the next frame
#     raw_capture.truncate(0)
#     print "shoot"
