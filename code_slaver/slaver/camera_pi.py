# -*- coding: utf-8 -*-

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Empty
import rospy
import numpy as np
# initialize the camera and grab a reference to the raw camera capture
def ask_callback():
    pass

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size = (640, 480))
camera.start_preview()
# allow the camera to warmup
time.sleep(2)
rospy.init_node('camera', anonymous=True)
pub_camera=rospy.Publisher("/pub_camera",CompressedImage,queue_size=1)
rospy.Subscriber("/ask_camera", Empty, ask_callback)

image=CompressedImage()
image.format="jpeg"
# while True:
#     image.header.stamp=rospy.Time.now()
#     camera.capture(image.data,format="bgr",resize=(640, 480))
#     pub_camera.publish(image)
#     # show the frame
#     #cv2.imshow("Frame", image)
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # show the frame
    image.header.stamp=rospy.Time.now()
    image.data = np.array(cv2.imencode('.jpg',frame.array)[1]).tostring()
    pub_camera.publish(image)
    print "shoot"

    raw_capture.truncate(0)

