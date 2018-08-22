
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs
import numpy as np
# import matplotlib.pyplot as plt
# from yolo import MC3YOLO
from PIL import Image

import point_control
import velocity_control

t=0
move_start=0

def detecttion(image,flag=True,isdraw=False):
    if not flag:
        return None
    im = Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB))
    model_path = './model_data/weights.h5'
    # anchor_path ='./model_data/yolo_anchors.txt'
    anchor_path ='./model_data/tiny_yolo_anchors.txt'
    class_path = './model_data/mc3_classes.txt'
    model = MC3YOLO(model_path,class_path,anchor_path,score=0.8,iou=0.5,image_size=(960,1280))

    res = model.detect_image(im)
    if not isdraw :
        return res
    img = np.asarray(im)
    for i in range(len(res)):
        print(res[i])
        color = ()
        c,s,x0,y0,x1,y1 = res[i]
        if c == 0:
            color = (255,0,0)
        if c == 1:
            color = (0,255,0)
        if c == 2:
            color = (0,0,255)
        cv2.rectangle(img,(x0,y0),(x1,y1),color=color,thickness=3)
    return img

def image_callback(msg):
    # print("Received an image!")
    global t
    # try:
    #     # Convert your ROS Image message to OpenCV2
    #     cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # except CvBridgeError, e:
    #     print(e)
    # else:
    #     # Save your OpenCV2 image as a jpeg 
    #     # cv2.imwrite('camera_image.jpeg', cv2_img)
    #    cv2.imshow("Image", cv2_img)
    # cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    # cv2.imshow("Image", cv2_img)
    # print 'wewewewe'
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # cv2.imwrite("Image.jpg", image_np)
    t=t+1
    if t%20==0:
        # cv2.imshow("image",image_np)
        res = detecttion(image_np,flag=True,isdraw=True)
        # cv2.imshow("result", image_np)
        cv2.imwrite(str(t)+'re1.jpg',image_np)
        cv2.waitKey(1000)

# if __name__ == '__main__':
#     mymove=point_control.GotoPoint()
#     # rospy.init_node('mc3_main', anonymous=True)
#     rate = rospy.Rate(10)
#     bridge = CvBridge()
#     rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
#     print 'hello'
#     while not rospy.is_shutdown():
#         print(point_control.msg)
#         (goal_x, goal_y, goal_z) = mymove.getkey()
#         # mymove.PointMove(0.5,0.1,0)
#         mymove.PointMove(goal_x, goal_y, goal_z)
#         (position, rotation) = mymove.get_odom()
#         print position, rotation
#         rate.sleep()
    # rospy.spin()

rospy.init_node('mc3_main', anonymous=True)
# mymove=point_control.GotoPoint()
velocity_control.init()
rate = rospy.Rate(10)
bridge = CvBridge()
rospy.Subscriber("/raspicam_node/image/compressed",sensor_msgs.msg.CompressedImage, image_callback)
print 'hello'
while not rospy.is_shutdown():

    if move_start==0:
        print(point_control.msg)
        (goal_x, goal_y, goal_z) = point_control.getkey()
        move_start=1
    else:
        if point_control.PointMove(goal_x, goal_y, goal_z)==4:
            move_start=0

    # print velocity_control.msg
    # (x,y)=velocity_control.getKey()
    # velocity_control.vel_control(x,y)
    rate.sleep()