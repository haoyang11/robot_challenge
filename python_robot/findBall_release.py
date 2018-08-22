
'''
File name: findBall_release.py
Funciont: find ball center
@Input: rgb picture
@Output: ballCenter
    - (0, 0) cannot find ball
    - (-1, -1) Error! input picture is not valid
    - (x, y) ball center

By Dongyan, 2018.7.31
'''



#from picamera.array import PiRGBArray    # if use pi to run real-time video, uncommmon this line
#from picamera import PiCamera            # if use pi to run real-time video, uncommmon this line
import time
import cv2
import os
import numpy as np
import math


FILE_PATH = "test_pic_1/"   # if start this .py directly, load pictures from FIEL_PATH


# properties of each contours.
class contourProperties:
    
    def __init__(self, p1, p2, contour, contour_area):
        self.rectangleBeginPoint = p1
        self.rectangleEndPoint = p2
        self.originalContour = contour
        self.originalContourArea = contour_area
        self.rectangleWidth = p2[0] - p1[0]
        self.rectangleHeight = p2[1] - p1[1]
        self.rectangleArea = self.rectangleWidth * self.rectangleHeight
        self.contourRectangleRatio = 1.0 * self.originalContourArea / self.rectangleArea
        self.widthHeightRatio = min(self.rectangleWidth/(self.rectangleHeight+1.0), self.rectangleHeight/(self.rectangleWidth+1.0))
        
    def showInfo(self):
        print "Position: ", self.rectangleBeginPoint, ", ", self.rectangleEndPoint, ", ratio: ", self.widthHeightRatio
        print "Contours: area: ", self.originalContourArea, ", rect_area:", self.rectangleArea, ", area_ratio: ", self.contourRectangleRatio



#------------------------------------ process fuction  --------------------------------
def doSomething(src):
    showInfoFlag  = False     # set 'True' to show details of the process.

    processTimeStart = time.time()
    
    if src is None:
        print "[Error]==> No picture is given to 'doSomething' function..."
        print "         exit process, and return (-1, -1)"
        return (-1,-1)
        
    ballCenter = (0, 0)
    
    isClose = False

    # parameter settings:
    # hsv threshold settings
    g_hLow, g_hHigh = 160, 14
    g_sLow, g_sHigh = 30, 255
    g_vLow, g_vHigh = 40, 200

    # open and close operation process settings
    g_openSize, g_closeSize = 7, 15

    # shape settings
    g_totalPixelRatio = 50          # larger than this value(percentage of total picture), think the ball is near
    g_convexThresh = 50             # smaller than this value(percentage), think the shape is strange, not a ball
    g_contourAreaRatioThresh = 2    # smaller than this value(permillage of total picture), too small area so not a ball
    g_widthHeightRatioThresh = 65   # the ratio of rectangle's width and height, smaller than this value means it's a bar not a ball
    g_ballPositionThresh = 50       # the ball must be at the bottom of picture. the area upper than this value(percentage of picture) cannot ba a ball
    
    
    # to hsv and find masks
    hsv = cv2.cvtColor(src, cv2.COLOR_BGR2HSV)
    hGray,sGray,vGray = cv2.split(hsv)
    
    hMask = np.zeros((hGray.shape[0],hGray.shape[1],1), np.uint8)
    sMask = np.zeros((sGray.shape[0],sGray.shape[1],1), np.uint8)
    vMask = np.zeros((vGray.shape[0],vGray.shape[1],1), np.uint8)

    # h is periodic from 0-255, so the upper boundary can smaller than lower boundary
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


    # open and close operation
    kernel = np.ones((7,7),np.uint8)
    opening = cv2.morphologyEx(commonMask, cv2.MORPH_OPEN, kernel)
    kernel = np.ones((15,15),np.uint8)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    #cv2.imshow("OpenClose", closing)


    
    img = closing
    imgRows, imgCols = img.shape[0], img.shape[1]
    
    # find weight center if too many white pixels
    x_total, y_total, pixelCnt = 0, 0, 0
    imgMoments = cv2.moments(img)   # find moments of pictures.
    x_total = imgMoments['m10'] / (imgMoments['m00'] + 1)   # find weight center in x axis. +1 to avoid /0
    y_total = imgMoments['m01'] / (imgMoments['m00'] + 1)
    pixelCnt = imgMoments['m00'] / 255  # 'm00' is moment, /255 to get area

    #print "x_,",x_total,", y_",y_total,"pix:,",pixelCnt
    
    # if pixels are larger than threshold, think the ball is near, return the weight center
    if pixelCnt > (imgRows*imgCols * g_totalPixelRatio * 0.01): 
        #center = (int((1.0*x_total) / pixelCnt), int((1.0*y_total) / pixelCnt))
        center = (int((1.0*x_total)), int((1.0*y_total)))
        if showInfoFlag == True:
            print "[Info]==>  The ball is very near. Center is: ", center
        ballCenter = center
        isClose = True
    else:   # check shape to find ball
        imgTmp, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_rectangle = []
        
        # find rectangles which enclose the contours
        for i in range(0, len(contours)):
            up, down, left, right = 999, 0, 999, 0
            contourWidth, contourHeight = 0, 0
            for j in range(0, len(contours[i])):
                p = contours[i][j][0]
                if p[1] < up:
                    up = p[1]
                if p[1] > down:
                    down = p[1]
                if p[0] < left:
                    left = p[0]
                if p[0] > right:
                    right = p[0]
            contour_rectangle.append((left,up,right,down))
            
        # save the parameters of rectangles. Position and area.
        contourPropertiesList = []
        for i in range(0, len(contours)):
            p1 = (contour_rectangle[i][0], contour_rectangle[i][1])
            p2 = (contour_rectangle[i][2], contour_rectangle[i][3])
            roi = img [ p1[1]:p2[1], p1[0]:p2[0] ]
            contour_area = cv2.countNonZero(roi)
            tmp = contourProperties(p1, p2, contours[i], contour_area)
            contourPropertiesList.append(tmp)
            
        possibleBallIndex = []  # store the possible ball index in all stored contoures 
        for i in range(len(contourPropertiesList)):
            cp = contourPropertiesList[i]
            if cp.originalContourArea > (1.0 * imgRows * imgCols * g_contourAreaRatioThresh) / 1000:
                if cp.widthHeightRatio > (1.0 * g_widthHeightRatioThresh) / 100:
                    if cp.contourRectangleRatio > (1.0 * g_convexThresh) / 100:
                        if (cp.rectangleBeginPoint[1] + cp.rectangleEndPoint[1]) / 2 > (1.0 * g_ballPositionThresh) * imgRows / 100:
                            possibleBallIndex.append(i)
        
        # pick the most possible ball's position 
        if len(possibleBallIndex) == 0:   # no possible ball in possibleBallIndex
            if showInfoFlag == True:
                print "[Info]==> Cannot find ball."
            ballCenter = (0, 0)
            
        elif len(possibleBallIndex) == 1: # only one element
            index = possibleBallIndex[0]
            cp = contourPropertiesList[index]
            if showInfoFlag == True:
                print "[Info]==> Find one ball at: ", ((cp.rectangleBeginPoint[0] + cp.rectangleEndPoint[0]) / 2, (cp.rectangleBeginPoint[1] + cp.rectangleEndPoint[1]) / 2)
            ballCenter = (int((cp.rectangleBeginPoint[0]+ cp.rectangleEndPoint[0]) / 2), int((cp.rectangleBeginPoint[1] + cp.rectangleEndPoint[1]) / 2))
        else:                       # many possible balls. Find the max one.
            maxArea, maxAreaIndex = 0, 0
            for i in range(0, len(possibleBallIndex)):
                if contourPropertiesList[possibleBallIndex[i]].originalContourArea >= maxArea:
                    maxArea = contourPropertiesList[possibleBallIndex[i]].originalContourArea
                    maxAreaIndex = possibleBallIndex[i]
            cp = contourPropertiesList[maxAreaIndex]
            if showInfoFlag == True:
                print "[Info]==> Many likely balls. Most possible position: ", \
                ((cp.rectangleBeginPoint[0] + cp.rectangleEndPoint[0]) / 2, (cp.rectangleBeginPoint[1] + cp.rectangleEndPoint[1]) / 2)
            ballCenter = (int((cp.rectangleBeginPoint[0]+ cp.rectangleEndPoint[0]) / 2), int((cp.rectangleBeginPoint[1] + cp.rectangleEndPoint[1]) / 2))

    if showInfoFlag == True:        
        print "[Result]==> ballCenter:", ballCenter, ", and is that close?: ", isClose
        print "[Time]==> Time consumed in process: ", round((time.time() - processTimeStart)*1000,2), "ms"
    return ballCenter, isClose

# --------------------------------    process fuction  --------------------------------

if __name__ == '__main__':

    keyValue = 1
    i = 0

    while True: # 'q' or Esc
        src = cv2.imread(FILE_PATH+str(i)+".jpg")
        ballCenter =  doSomething(src)
        i += 1

