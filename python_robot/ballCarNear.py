
'''
Check whether the car or ball is near.

How it work:
1. Load a picture.
2. Use isCarNear to check whether near a car
3. Use isBallNear to check whether near a ball
4. Return results.
5. Change original parameter of checkBall/CarNear_release.py

Dong yan. 2018.8.8


'''

import cv2
# import by dongyan
from checkBallNear_release import isBallNear
from checkCarNear_release import isCarNear




i = 0
FILE_PATH = "rubishbin/"

while True: # 'q' or Esc

    img = cv2.imread(FILE_PATH+str(i)+".jpg")

    car_near = isCarNear(img)
    ball_near = isBallNear(img)

    print "---------------"
    print "No.", i, ", Car:", car_near, ", Ball: ", ball_near

    cv2.imshow("Source", img)
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



