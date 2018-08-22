# -*- coding: utf-8 -*-

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import rospy
from service_demo.srv import *

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size = (1280, 720))

# allow the camera to warmup
time.sleep(1)

# capture frames from the camera
i=0
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # show the frame
    image = frame.array
    #cv2.imshow("Frame", image)
    cv2.imwrite(str(i)+"_ball_top.jpg",image)
    i=i+1

    # clear the stream in preparation for the next frame
    raw_capture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord("q"):
        cv2.destroyAllWindows()
        break

import rospy
from service_demo.srv import *

def server_srv():
    # 初始化节点，命名为 "greetings_server"
    rospy.init_node("greetings_server")
    # 定义service的server端，service名称为"greetings"， service类型为Greeting
    # 收到的request请求信息将作为参数传递给handle_function进行处理
    s = rospy.Service("greetings", Greeting, handle_function)
    rospy.loginfo("Ready to handle the request:")
    # 阻塞程序结束
    rospy.spin()

# Define the handle function to handle the request inputs
def handle_function(req):
    # 注意我们是如何调用request请求内容的，与前面client端相似，都是将其认为是一个对象的属性，通过对象调用属性，在我们定义
    # 的Service_demo类型的service中，request部分的内容包含两个变量，一个是字符串类型的name，另外一个是整数类型的age
    rospy.loginfo( 'Request from %s with age %d', req.name, req.age)
    # 返回一个Service_demoResponse实例化对象，其实就是返回一个response的对象，其包含的内容为我们再Service_demo.srv中定义的
    # response部分的内容，我们定义了一个string类型的变量，因此，此处实例化时传入字符串即可
    return GreetingResponse("Hi %s. I' server!"%req.name)


from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
