
'''
Filename: checkBallNear_debug.py
Describe: 
    check whether this is a ball near the camera, with toorbar to change threshold
    after changing the threshold, new threshold can be used in checkBallNear_release.py

Attention: Do not run this .py in pi, because pi cannot show pictures.

@Fuction: isBallNear()
@Input: a rgb picture and size should be (whatever, 480)
@Return: 'True' or 'False', or None (if @Input is none)

How it works:
1. rgb to hsv, use H to get mask picture, from lower to upper boundary. (240 and 20 by default)
2. get the bottom bar of picture (default: 400-480)
3. divide the bar to some vertical rectangles (default: 20)

4. check each rectangle, if the "ball" part is larger than a percentage (default:50%) of that rectangle area, that rectangle is marked
5. count all marked rectangles. if the marked number is larger than a value (default:16), think the ball is near
6. return True or False and output the marked number.

Dong Yan, 2018.7.31


1.Change the bgr to hsv, use all H,S,V channels to selet potential "ball" part.
    Attention: H:240-20, S:30-255, V:30-200 (larger than 30, cannot see lower part of ball; smaller, can get balck )
Dongyan 2018.8.8 
'''


#from picamera.array import PiRGBArray    #if use pi, uncommmon this line
#from picamera import PiCamera            #if use pi, uncommmon this line
import time
import cv2
import numpy as np

FILE_PATH = "rubishbin/"

#------------------------------------ process fuction  --------------------------------
def isBallNear(src):
    t = time.time()

    showInfoFlag = 1

    if src is None:
        print "[Error]==> No picture is given to 'isBallNear' function..."
        return

    global g_hLow, g_hHigh, g_sLow, g_sHigh, g_vLow, g_vHigh
    
    # to hsv and find masks
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    hGray,sGray,vGray = cv2.split(hsv)
    
    hMask = np.zeros((hGray.shape[0],hGray.shape[1],1), np.uint8)
    sMask = np.zeros((sGray.shape[0],hGray.shape[1],1), np.uint8)
    vMask = np.zeros((vGray.shape[0],hGray.shape[1],1), np.uint8)

    if g_hLow >= g_hHigh:
        ret, thresh1 = cv2.threshold(hGray, g_hLow, 255, cv2.THRESH_BINARY)
        ret, thresh2 = cv2.threshold(hGray, g_hHigh, 255, cv2.THRESH_BINARY_INV)
        hMask = thresh1 | thresh2
    else:
        ret, thresh1 = cv2.threshold(hGray, g_hLow, 255, cv2.THRESH_BINARY)
        ret, thresh2 = cv2.threshold(hGray, g_hHigh, 255, cv2.THRESH_BINARY_INV)
        hMask = thresh1 & thresh2 

    ret, thresh1 = cv2.threshold(sGray, g_sLow, 255, cv2.THRESH_BINARY)
    ret, thresh2 = cv2.threshold(sGray, g_sHigh, 255, cv2.THRESH_BINARY_INV)
    sMask = thresh1 & thresh2
    
    ret, thresh1 = cv2.threshold(vGray, g_vLow, 255, cv2.THRESH_BINARY)
    ret, thresh2 = cv2.threshold(vGray, g_vHigh, 255, cv2.THRESH_BINARY_INV)
    vMask = thresh1 & thresh2
    
    commonMask = hMask & sMask & vMask

    continueImg = commonMask


    showMat = src.copy()
    imgRows, imgCols = src.shape[0], src.shape[1]
    
    global g_linePosition, g_divideNumber, g_checkThresh
    
    markedRectangleNumber = 0
    markedRectangleList = []
    rectangleWidth, rectangleHeight = (int)(imgCols/g_divideNumber), (imgRows - g_linePosition)
    for i in range(0,g_divideNumber):
        cv2.rectangle(showMat, (i* rectangleWidth, g_linePosition),((i+1)* rectangleWidth, imgRows-1),(0,255,0),2)
        totalPixel = rectangleWidth * rectangleHeight
        pixelCnt = cv2.countNonZero(continueImg [g_linePosition:imgRows, i*rectangleWidth:(i+1)*rectangleWidth])
        
        if pixelCnt > (float(totalPixel) * g_checkThresh / 100.0):
             cv2.rectangle(showMat, (i* rectangleWidth, g_linePosition),((i+1)* rectangleWidth, imgRows-1),(0,0,255),-1)
             markedRectangleNumber += 1
             markedRectangleList.append(i)

    # should uncommon this one.
    # cv2.imshow("Result", showMat)


    # for i in range(len(markedRectangleList)):
        # print markedRectangleList[i],
    # print " is marked. Number: ", markedRectangleNumber

    print "Time used in checkBallNear.-- ", int((time.time()-t)*1000),"ms"
    if markedRectangleNumber >= 10:
        # print "[Car Result] ==> return [ True ]"
        return True
    else: 
        # print "[Car Result] ==> return [ False]"
        return False



# --------------------------------    process fuction  --------------------------------



keyValue = 1
i = 0

g_hLow, g_hHigh = 240, 20
g_sLow, g_sHigh = 30, 255
g_vLow, g_vHigh = 30, 200
g_linePosition, g_divideNumber, g_checkThresh = 400, 20, 50

import os



if __name__ == '__main__':

    im_dir = "./pic8.16"
    im_name = os.listdir(im_dir)
    for name in im_name:
        if '.jpg' not in name:
            continue

        im_path = os.path.join(im_dir, name)
        src = cv2.imread(im_path, 1)
        ballCenter =  isBallNear(src)
        isBallNear(src)

        cv2.imshow("Source", src)
        keyValue = cv2.waitKey(0)


    '''
    while True: # 'q' or Esc
        src = cv2.imread(FILE_PATH+str(i)+".jpg")
        
        print "No. ",i
        ballCenter =  isBallNear(src)
        isBallNear(src)

        cv2.imshow("Source", src)
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
    '''



