
'''
findBoundaryDistance_release.py:
    find boundary of the court

@input: pictures from 'FILE_PATH', 0.jpg - end
@output: distance from the boundary, 0(not found), -1(error)

How it works:
1. Change rgb 2 hsv, find h-mask by two threshold, 40 and 80 by default
2. Open and close operate to elimate the center line and other noises.
3. Check whether has boundary, by counting the white pixels in the bottom of picture. 
    If larger the threshold (20% of bar), has a bar, then try to find; if not, doesn't have a boundary.
4. Use canny to extract the edges, then count pixels of horizontal bar, find the maximum of bottom half of picture.
5. Draw the line to show the maximum one, and change it to distance using fitting curve (from Matlab)
6. Return the distance


last change: 2018.8.5 
Dongyan


'''


#from picamera.array import PiRGBArray    #if use pi, uncommmon this line
#from picamera import PiCamera            #if use pi, uncommmon this line
import time
import cv2
import numpy as np

FILE_PATH = "door/"


def findBoundaryDistance(src):
    cv2.imshow("Source", src)
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
    
    #cv2.imshow("H", hGray)

    showMat = src.copy()
    imgRows, imgCols = src.shape[0], src.shape[1]
    
    #cv2.imshow("Source", src)
    #cv2.imshow("H-mask", hMask)
    
    showMat = src.copy()
    imgRows, imgCols = src.shape[0], src.shape[1]
    
    kernel = np.ones((15,15),np.uint8)
    opening = cv2.morphologyEx(hMask, cv2.MORPH_OPEN, kernel)
    kernel = np.ones((15,15),np.uint8)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    #cv2.imshow("OpenClose", closing)

    # bottomBar is used to check "Very close to wall" conditions. If bottom bar has many white pixels, think is near wall.
    bottomBar = closing[ (imgRows-g_barWidth-1):(imgRows-1) , 0:(imgCols -1) ]
    th = 1.0 * bottomBar.shape[0] * bottomBar.shape[1] * g_bottomBarThresh / 100.0
    nonZero = cv2.countNonZero(bottomBar);
    if cv2.countNonZero(bottomBar) < th:
		cv2.line(showMat, (0, 0), (imgCols - 1, imgRows - 1), (0, 0, 255), 5)
		print "[Result] Cannot find boundary Lines..."
		return 0

    # if has a boundary, try to find
    cannyImg = cv2.Canny(closing, 3, 9, 3)
    #cv2.imshow("Canny", cannyImg);
    
    maxIndex, maxPixelCnt = 0, 0
    # for i in range(0, (int)(imgRows/2/g_barWidth)):
    for i in range(0, (int)(260/g_barWidth)):
        pixelCnt = cv2.countNonZero(cannyImg[(imgRows - (i+1) * g_barWidth):(imgRows - i*g_barWidth - 1) , 0:(imgCols - 1)])
        if pixelCnt > maxPixelCnt:
            maxPixelCnt = pixelCnt
            maxIndex = i
        #cv2.rectangle(showMat, (0, imgRows - (i+1)*g_barWidth), (imgCols-1, imgRows - i*g_barWidth - 1), (32, 0, 0), 1)
    

    print "Max i:,", i
    linePosition = (int)((imgRows - (maxIndex + 1)*g_barWidth) + (imgRows - maxIndex*g_barWidth - 1) )/2
    #cv2.line(showMat, (0, linePosition), (imgCols, linePosition), (0, 0, 128), 2)
    #cv2.imshow("Result", showMat);
    
    
    if linePosition > imgRows or linePosition < imgRows/2 :
        print "[Error] Unexpected line position......"
        return -1
    
    distance = (1.96 * linePosition + 6280)/(linePosition - 252) / 100        # fitting curve
    print "[Result] Line position: ", linePosition, ", distance: ", distance
    return distance 


keyValue = 1
g_hLow, g_hHigh = 40, 80
g_bottomBarThresh, g_barWidth = 20, 5
g_pictureNumber = 0

if __name__ == '__main__':

    i = 0   
    while True: # 'q' or Esc
        src = cv2.imread(FILE_PATH+str(i)+".jpg")
        
        print "No. ",i
        findBoundaryDistance(src)

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



