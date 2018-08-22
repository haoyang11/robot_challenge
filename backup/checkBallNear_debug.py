
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
7. show No.1-4's toolbar and judge the threshold

Dong Yan, 2018.7.31
'''


#from picamera.array import PiRGBArray    #if use pi, uncommmon this line
#from picamera import PiCamera            #if use pi, uncommmon this line
import time
import cv2
import numpy as np

FILE_PATH = "nearBallPic/"


#   ----------------------- toolbar------------------------------------
def onTrackbar(value):

    hLow = cv2.getTrackbarPos("H min", "H-mask")
    hHigh = cv2.getTrackbarPos("H max", "H-mask")    
    global g_hLow, g_hHigh
    g_hLow, g_hHigh = hLow, hHigh
    
    linePosition = cv2.getTrackbarPos("line", "Result")
    divideNumber = cv2.getTrackbarPos("number", "Result")
    checkThresh = cv2.getTrackbarPos("thresh", "Result")
    global g_linePosition, g_divideNumber, g_checkThresh
    g_linePosition, g_divideNumber, g_checkThresh = linePosition, divideNumber, checkThresh
    
    isBallNear(src)
 
    return
#   ----------------------- toolbar------------------------------------

#------------------------------------ process fuction  --------------------------------
def isBallNear(src):

    showInfoFlag = 1

    if src is None:
        print "[Error]==> No picture is given to 'isBallNear' function..."
        return

    global g_hLow, g_hHigh
    
    # to hsv and find masks
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    hGray,sGray,vGray = cv2.split(hsv)
    
    hMask = np.zeros((hGray.shape[0],hGray.shape[1],1), np.uint8)

    if g_hLow >= g_hHigh:
        ret, thresh1 = cv2.threshold(hGray, g_hLow, 255, cv2.THRESH_BINARY)
        ret, thresh2 = cv2.threshold(hGray, g_hHigh, 255, cv2.THRESH_BINARY_INV)
        hMask = thresh1 | thresh2
    else:
        ret, thresh1 = cv2.threshold(hGray, g_hLow, 255, cv2.THRESH_BINARY)
        ret, thresh2 = cv2.threshold(hGray, g_hHigh, 255, cv2.THRESH_BINARY_INV)
        hMask = thresh1 & thresh2 
    
    cv2.imshow("H", hGray)
    cv2.imshow("H-mask", hMask)

    showMat = src.copy()
    imgRows, imgCols = src.shape[0], src.shape[1]
    
    global g_linePosition, g_divideNumber, g_checkThresh
    
    markedRectangleNumber = 0
    markedRectangleList = []
    rectangleWidth, rectangleHeight = (int)(imgCols/g_divideNumber), (imgRows - g_linePosition)
    for i in range(0,g_divideNumber):
        cv2.rectangle(showMat, (i* rectangleWidth, g_linePosition),((i+1)* rectangleWidth, imgRows-1),(0,255,0),2)
        totalPixel = rectangleWidth * rectangleHeight
        pixelCnt = cv2.countNonZero(hMask [g_linePosition:imgRows, i*rectangleWidth:(i+1)*rectangleWidth])
        
        if pixelCnt > (float(totalPixel) * g_checkThresh / 100.0):
             cv2.rectangle(showMat, (i* rectangleWidth, g_linePosition),((i+1)* rectangleWidth, imgRows-1),(0,0,255),-1)
             markedRectangleNumber += 1
             markedRectangleList.append(i)

    cv2.imshow("Result", showMat)

# --------------------------------    process fuction  --------------------------------



keyValue = 1
i = 0

g_hLow, g_hHigh = 240, 20

cv2.namedWindow("Source")
cv2.namedWindow("H")
cv2.namedWindow("H-mask")
cv2.namedWindow("Result")
cv2.createTrackbar("H min", "H-mask", 240, 255, onTrackbar);
cv2.createTrackbar("H max", "H-mask", 20, 255, onTrackbar);


g_linePosition, g_divideNumber, g_checkThresh = 400, 20, 50
cv2.createTrackbar("line", "Result", 400, 479, onTrackbar);
cv2.createTrackbar("number", "Result", 20, 20, onTrackbar);
cv2.createTrackbar("thresh", "Result", 50, 100, onTrackbar);


g_pictureNumber = 0

if __name__ == '__main__':

    while True: # 'q' or Esc
        src = cv2.imread(FILE_PATH+str(i)+".jpg")
        
        print "No. ",i
        ballCenter =  isBallNear(src)
        g_pictureNumber += 1
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

    
if __name__ != '__main__':
    print "[Error] ==> Cannot run checkBallNear_debug.py as a function in pi"








