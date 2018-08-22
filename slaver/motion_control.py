#!/usr/bin/env python

'''
Dong Yan changed 8.1

1. if input x>0, rotate and move
2. if input x<0 while -0.001<y<0.001, move backward
3. Use distance error < 0.5 as well as vector to judge stop.

4.Used new parameter 8.3

'''
#




import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty,Float64MultiArray,Bool
from sensor_msgs.msg import Imu
import tf
from math import radians, copysign, sqrt, pow, pi, atan2,cos,sin
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import os

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

point_msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""
vel_msg = """
Control Your TurtleBot3!
---------------------------
Velocity(0~1):
        +X
   +Y    s    -Y
        -X

X :linear velocity(+front,-back)
Y :angular velocity(+left,-right)

space key, s : force stop

CTRL-C to quit
"""

# tb_name="7"
tb_name=str(os.getenv('TB_NAME'))

stage_first=1
rev_odom=1
rev_imu=1

empty_msg=Empty()
rotation = 0
position = []
global_pos=[0.0,0.0,0.0]
global_rot=0.0
rpy_start=[0.0,0.0,0.0]

point_state=False
rotate_state=False

def point_state_callback(msg):
    global point_state
    point_state=msg.data

def rotate_state_callback(msg):
    global rotate_state
    rotate_state=msg.data

def init():
    global pub_motion_control_reset,cmd_vel,position,move_cmd,odom_frame,pub_reset,rpy_start,tf_listener,base_frame,pub_point_control,pub_rotate_control

    print "tb_name:",tb_name
    rospy.Subscriber("/tb"+tb_name+"/imu", Imu, imu_callback)
    rospy.Subscriber("/tb"+tb_name+"/odom", Odometry, odometry_callback)
    # pub_reset = rospy.Publisher("/reset",Empty,queue_size=1)
    pub_motion_control_reset = rospy.Publisher("/tb"+tb_name+"/motion_control_reset",Empty,queue_size=1)
    rospy.on_shutdown(shutdown)
    cmd_vel = rospy.Publisher("/tb"+tb_name+'/cmd_vel',Twist,queue_size=5)

    pub_point_control=rospy.Publisher("/tb"+tb_name+"/point_control", Float64MultiArray,queue_size=1)
    rospy.Subscriber("/tb"+tb_name+"/point_state", Bool,point_state_callback )
    pub_rotate_control=rospy.Publisher("/tb"+tb_name+"/rotate_control", Float64MultiArray,queue_size=1)
    rospy.Subscriber("/tb"+tb_name+"/rotate_state", Bool, rotate_state_callback)

    position = Point()
    move_cmd = Twist()
    r = rospy.Rate(100)
    # tf_listener = tf.TransformListener()
    odom_frame = 'odom'
    rospy.sleep(1)
    reset()
    rpy_start=get_imu()
    
    print "start",rpy_start
    # try:
    #     tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    #     base_frame = 'base_footprint'
    # except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #     try:
    #         tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
    #         base_frame = 'base_link'
    #     except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #         rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
    #         rospy.signal_shutdown("tf Exception")


def odometry_callback(msg):
    global rev_odom,get_trans,get_odom_orientation
    # print "i rev"
    if rev_odom==0:
        get_trans=msg.pose.pose.position
        get_odom_orientation=msg.pose.pose.orientation
        rev_odom=1

def imu_callback(msg):
    global rev_imu,get_imu_orientation
    # print "i rev"
    if rev_imu==0:
        get_imu_orientation=msg.orientation
        rev_imu=1

def PointMove(x,y,z):
    global pub_point_control,point_state
    point_goal = Float64MultiArray()
    point_goal.data=x,y,z
    point_state=False
    pub_point_control.publish(point_goal)
    while point_state==False:
        pass
    print "Finish point"

def rotateTo2(angle):

    global pub_rotate_control,rotate_state
    rotate_goal = Float64MultiArray()
    rotate_goal.data.append(angle)
    rotate_state=False
    pub_rotate_control.publish(rotate_goal)
    
    print "Begin rotate..." 
    t = time.time()

    while rotate_state==False:
        pass
    print "Finish rotate"
    print "Time used in PC: ", round((time.time()-t)*1000), "ms"




def point_getkey():
    x, y, z = raw_input("| x | y | z |\n").split()
    if x == 's':
        shutdown()
    x, y, z = [float(x), float(y), float(z)]
    return x, y, z

def get_odom():
    global tf_listener,base_frame
    global rev_odom,get_trans,get_odom_orientation
    rev_odom=0
    while rev_odom!=1:
        pass
        # print "wait odom" 
    (_, _, get_odom_yaw) = tf.transformations.euler_from_quaternion([get_odom_orientation.x, get_odom_orientation.y, get_odom_orientation.z, get_odom_orientation.w])
    return (get_trans,get_odom_yaw/2)

    # try:
    #     (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
    #     rotation = euler_from_quaternion(rot)

    # except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #     rospy.loginfo("TF Exception")
    #     return

    return (Point(*trans), rotation[2])
def get_imu():
    global rev_imu,get_imu_orientation,rpy_start
    rev_imu=0
    while rev_imu!=1:
        pass
        # print "wait odom"
    get_rpy = list(tf.transformations.euler_from_quaternion([get_imu_orientation.x, get_imu_orientation.y, get_imu_orientation.z, get_imu_orientation.w]))
    for i in range(3):
        get_rpy[i]=(get_rpy[i])*180/pi -rpy_start[i]
        if(get_rpy[i]>180):
            get_rpy[i]=get_rpy[i]-360
        if(get_rpy[i]<-180):
            get_rpy[i]=get_rpy[i]+360

    return get_rpy

def shutdown():
    global cmd_vel
    cmd_vel.publish(Twist())
    rospy.sleep(1)

# def zhijiao(x,y,z):#0 heng 1 shu 2 finish
#     zj_stage=0 #0 heng 1 shu 2 finish
#     if zj_stage==0:
#         if PointMove(0,y-0.01,0)==4:
#             zj_stage=1
#     if zj_stage==1:
#         if PointMove(x,y,z)==4:
#             zj_stage=2
#     return zj_stage#0 heng 1 shu 2 finish

def piRange(angle):
    if angle > pi:
        angle = angle - 2*pi
    elif angle < -pi:
        angle = angle + 2*pi
    return  angle
def reset():
    global pub_reset,pub_motion_control_reset,global_pos,global_rot,position,rotation
    global_pos[0]=global_pos[0]+position.x
    global_pos[1]=global_pos[1]+position.y
    global_pos[2]=global_pos[2]+position.z
    global_rot=global_rot+rotation
    global_rot=global_rot%2
    while global_rot< -1:
        global_rot=global_rot+2
    while global_rot> 1:
        global_rot=global_rot-2
    # pub_reset.publish(empty_msg)
    pub_motion_control_reset.publish(empty_msg)
    # while position.x>0.01 or position.y>0.01:
    #     (position, rotation) = get_odom()
    # new line by Dong yan

    # print "Reset odom complete"

def vel_getKey():
    x, y= raw_input("| x | y |\n").split()
    if x == 's' or y == 's':
        return 's','s'
    return x, y

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel
def vel_control(x,y):
    global cmd_vel
    if x == 's':
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        return 0
    else:
        x, y = [float(x), float(y)]

    twist = Twist()

    # control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    # twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

    # control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    # twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
    # twist.linear.x=checkLinearLimitVelocity(x*BURGER_MAX_LIN_VEL);twist.linear.y = 0.0;twist.linear.z = 0.0
    # twist.angular.z=checkAngularLimitVelocity(y*BURGER_MAX_ANG_VEL);twist.angular.x = 0.0; twist.angular.y = 0.0
    twist.linear.x=x
    twist.angular.z=y
    cmd_vel.publish(twist)
    return 1
def test():
    #####point######
    # (position, rotation) = get_odom()
    # t = time.time()
    # print "<Start>  poisition, ",position,"rotation, ",rotation
    # (goal_x, goal_y, goal_z) = point_getkey()
    # PointMove(goal_x, goal_y, goal_z)
    # print "<End> poisition, ",position,"rotation, ",rotation
    # print "Time: ", round((time.time()-t)*1000), "ms"

    #####rotate######
    # angle=float(raw_input("angle\n"))
    # rotateTo2(angle)

    (goal_x, goal_y, goal_z) = point_getkey()
    PointMove(goal_x, goal_y, goal_z)

    ######odom######
    # (position, rotation) = get_odom()
    # print "rotation=",rotation

if __name__ == '__main__':

    move_start=0;
    rospy.init_node("tb"+tb_name+'mc3_main', anonymous=True)
    init()
    print 'hello'
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        test()
        rate.sleep()
