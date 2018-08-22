
'''
Filename: checkBallNear_release.py
Describe: check whether this is a ball near the camera

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
'''



#from picamera.array import PiRGBArray
#from picamera import PiCamera
import time
import cv2
import numpy as np


#------------------------------------ process fuction  --------------------------------
def isBallNear(src):

    showInfoFlag = False    # set 'True' to show details of process.
    
    if src is None:
        print "[Error] ==> No picture is given to 'isBallNear' function, and return 'None'. (in checkBallNear_release.py)"
        return

    # these threshold can be changed
    hLow, hHigh = 240,20
    
    # to hsv and find masks
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    hGray,sGray,vGray = cv2.split(hsv)
    
    hMask = np.zeros((hGray.shape[0],hGray.shape[1],1), np.uint8)
    if hLow >= hHigh:
        ret, thresh1 = cv2.threshold(hGray, hLow, 255, cv2.THRESH_BINARY)
        ret, thresh2 = cv2.threshold(hGray, hHigh, 255, cv2.THRESH_BINARY_INV)
        hMask = thresh1 | thresh2
    else:
        ret, thresh1 = cv2.threshold(hGray, hLow, 255, cv2.THRESH_BINARY)
        ret, thresh2 = cv2.threshold(hGray, hHigh, 255, cv2.THRESH_BINARY_INV)
        hMask = thresh1 & thresh2 

    imgRows, imgCols = src.shape[0], src.shape[1]

    # threshold that can be changed.
    var_linePosition = 400      # selected bar is below this line. from 400-480 pixels by default
    var_divideNumber = 20       # the bar is divided to rectangles of this number. Defualt 20, so each is (640/20=32) piexls width
    var_checkThresh = 50        # percentage of ball area. If ("highlighted" pixels / rectangle area) is large than this value, think the rectangle is marked
    
    markedRectangleNumber = 0   # count how many rectanlge is marked.
    markedRectangleList = []

    rectangleWidth, rectangleHeight = (int)(imgCols/var_divideNumber), (imgRows - var_linePosition)
    for i in range(0,var_divideNumber):
        totalPixel = rectangleWidth * rectangleHeight
        pixelCnt = cv2.countNonZero(hMask [var_linePosition:imgRows, i*rectangleWidth:(i+1)*rectangleWidth])
        if pixelCnt > (float(totalPixel) * var_checkThresh / 100.0):
            markedRectangleNumber += 1
            markedRectangleList.append(i)   # record the index

    if showInfoFlag == True:
        print "[Info]==> No:",
        for i in range(len(markedRectangleList)):
            print markedRectangleList[i],
        print " is marked. Number: ", markedRectangleNumber

    # if the marked rectangles number is larger than a value(default:16), think the ball is near
    if markedRectangleNumber >= 16:
        print "[Result] ==> return [ True ]"
        return True
    else: 
        print "[Result] ==> return [ False]"
        return False

# --------------------------------    process fuction  --------------------------------



if __name__ == '__main__':
    
    FILE_PATH = 'nearBallPic/'
    print "[Attention] ==> checkBallNear_release.py is running as 'main'. Load pictures from file: ", FILE_PATH

    keyValue = 1
    i = 0

    while True: # 'q' or Esc

        g_src = cv2.imread(FILE_PATH+str(i)+".jpg")
        print "Picture no:", i
        
        ballCenter =  isBallNear(g_src)
        
        cv2.imshow("Source", g_src)
        keyValue = cv2.waitKey(0)
        if keyValue == 113 or keyValue == 27:   # 'q' or 'Esc'
            break
        elif keyValue == 97: # 'a'
            i -= 1 
            if i < 0:
                i = 0
        elif keyValue == 98: # 'b' for beginning one
            i = 0
        else:
            i += 1









