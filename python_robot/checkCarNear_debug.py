
'''
checkCarNear_debug.py

@Input: a 640x480 bgr picture
@Output: True or False

How it works:
1. Change a bgr 2 gray and use a threshold(30 by default) to selet dark part.
2. Draw a mask picture, a semi-circle on the bottom of picture
3. Count the white pixels in semi-circle region, if larger than a proportion(default: 40%), return True


Dong Yan, 2018.8.7

'''


#from picamera.array import PiRGBArray    #if use pi, uncommmon this line
#from picamera import PiCamera            #if use pi, uncommmon this line
import time
import cv2
import numpy as np

FILE_PATH = "rubishbin/"


#   ----------------------- toolbar------------------------------------
def onTrackbar(value):

    global g_binaryThreshold, g_maskRadius, g_isCarThreshold
    global g_roiMask 

    thresh = cv2.getTrackbarPos("thresh", "binary")
    radius = cv2.getTrackbarPos("radius", "binary")
    carThresh = cv2.getTrackbarPos("isCar", "binary")

    g_binaryThreshold = thresh
    g_maskRadius = radius
    g_isCarThreshold = carThresh

    emptyMat = np.zeros((480, 640), np.uint8)
    g_roiMask = cv2.circle(emptyMat, (320, 480), g_maskRadius, 255, -1)

    print "New parameter: thresh,", g_binaryThreshold,", radius:,", g_maskRadius, ", carThresh,", g_isCarThreshold

    isCarNear(g_src)

    return
#   ----------------------- toolbar------------------------------------

#------------------------------------ process fuction  --------------------------------
def isCarNear(src):
    global g_binaryThreshold, g_maskRadius, g_isCarThreshold
    global g_roiMask

    if src is None:
        print "[Error]==> No picture is given to 'isBallNear' function..."
        return

    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray", gray)
    ret, binImg = cv2.threshold(gray, g_binaryThreshold, 255, cv2.THRESH_BINARY_INV)

    # showMat = cv2.copy(binImg)
    showMat = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
    # cv2.circle(showMat, (320, 480), g_maskRadius, (0, 0, 255), 2)
    cv2.imshow("binary", showMat)

    # imshow
    # cv2.imshow("mask", g_roiMask)
    # cv2.waitKey(0) 
    nonZeroCnt = cv2.countNonZero(binImg)
    # roi =  binImg & g_roiMask

    # cv2.imshow("ROI", roi)
    print "Non zero in roi:", nonZeroCnt
    th = 640*480 * g_isCarThreshold /100
    if nonZeroCnt > th:
        print "is car threshold: ", th
        # print "The car is near..."
        return True
    else:
        return False



keyValue = 1
i = 0

g_binaryThreshold = 10
g_maskRadius = 200
g_isCarThreshold = 27

cv2.namedWindow("gray")
cv2.namedWindow("binary")
cv2.createTrackbar("thresh", "binary", g_binaryThreshold, 255, onTrackbar);
cv2.createTrackbar("radius", "binary", g_maskRadius, 320, onTrackbar);
cv2.createTrackbar("isCar", "binary", g_isCarThreshold, 100, onTrackbar);

g_pictureNumber = 0
g_src = None


g_roiMask = np.zeros((480, 640), np.uint8)
cv2.circle(g_roiMask, (320, 480), g_maskRadius, 255, -1)


if __name__ == '__main__':
    global g_src

    while True: # 'q' or Esc
        g_src = cv2.imread(FILE_PATH+str(i)+".jpg")
        
        print "No. ",i
        
        g_pictureNumber += 1

        cv2.imshow("Source", g_src)
        isCarNear(g_src)
        keyValue = cv2.waitKey(0)
        
        if keyValue == 113 or keyValue == 27:  #q or "Esc"
            break
        elif keyValue == 97:
            i = i-1
            if i < 0:
                i = 0
        elif keyValue ==98:
            i = 0
        else:
            i = i+1

    
if __name__ != '__main__':
    print "[Error] ==> Cannot run checkBallNear_debug.py as a function in pi"








