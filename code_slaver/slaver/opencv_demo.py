# -*- coding: utf-8 -*-

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size = (640, 480))

# allow the camera to warmup
time.sleep(1)

# capture frames from the camera
i=0
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # show the frame
    image = frame.array
    #cv2.imshow("Frame", image)
    cv2.imwrite("0807/"+str(i)+"_ball_top.jpg",image)
    i=i+1

    # clear the stream in preparation for the next frame
    raw_capture.truncate(0)
    cv2.waitKey(0)
    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break
