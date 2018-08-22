
'''
checkCarNear_release.py

@Input: a 640x480 bgr picture
@Output: True or False

How it works:
1. Change a bgr 2 gray and use a threshold(10 by default) to selet dark part.
2. Count the white pixels in the whole picture. If larger than 80000 pixels, think nearCar

Dong Yan 2018.8.8
'''

import cv2

FILE_PATH = "rubishbin/"

g_binaryThreshold = 10
g_whilePiexlThreshold = 80000


#------------------------------------ process fuction  --------------------------------
def isCarNear(src):
    global g_binaryThreshold, g_whilePiexlThreshold
    global g_roiMask

    if src is None:
        print "[Error]==> No picture is given to 'isBallNear' function..."
        return

    gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
    # cv2.imshow("gray", gray)
    ret, binImg = cv2.threshold(gray, g_binaryThreshold, 255, cv2.THRESH_BINARY_INV)

    # showMat = cv2.copy(binImg)
    showMat = cv2.cvtColor(binImg, cv2.COLOR_GRAY2BGR)
    # cv2.imshow("binary", showMat)

    # imshow
    # cv2.imshow("mask", g_roiMask)
    # cv2.waitKey(0) 
    
    # roi =  binImg & g_roiMask
    nonZeroCnt = cv2.countNonZero(binImg)

    # cv2.imshow("ROI", roi)
    # print "Thresh:", areaThresh, "Now: ",  nonZeroCnt

    if nonZeroCnt > g_whilePiexlThreshold:
        # print "<Return>   - True -"
        return True
    else:
        # print "<Return>   - False -"
        return False





i = 0
g_src = None

if __name__ == '__main__':
    global g_src

    while True: # 'q' or Esc
        g_src = cv2.imread(FILE_PATH+str(i)+".jpg")
        
        print "No. ",i

        cv2.imshow("Source", g_src)
        isCarNear(g_src)
        keyValue = cv2.waitKey(500)
        
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






