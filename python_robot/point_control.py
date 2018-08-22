#!/usr/bin/env python

'''
Dong Yan changed 8.1

1. if input x>0, rotate and move
2. if input x<0 while -0.001<y<0.001, move backward
3. Use distance error < 0.5 as well as vector to judge stop.


'''
#




import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
import tf
from math import radians, copysign, sqrt, pow, pi, atan2,cos,sin
from tf.transformations import euler_from_quaternion
import numpy as np
import time

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

stage_first=1
rev_odom=1
rev_imu=1

empty_msg=Empty()
rotation = 0
position = []
global_pos=[0.0,0.0,0.0]
global_rot=0.0
rpy_start=[0.0,0.0,0.0]


def init():
    global cmd_vel,position,move_cmd,odom_frame,pub_reset,rpy_start,tf_listener,base_frame
    # rospy.init_node('turtlebot3_pointop_key', anonymous=False)
    rospy.Subscriber("/imu", Imu, imu_callback)
    # rospy.Subscriber("/odom", Odometry, odometry_callback)
    pub_reset = rospy.Publisher("/reset", Empty, queue_size=1)
    rospy.on_shutdown(shutdown)
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    position = Point()
    move_cmd = Twist()
    r = rospy.Rate(100)
    tf_listener = tf.TransformListener()
    odom_frame = 'odom'
    rospy.sleep(1)
    reset_odom()
    rpy_start=get_imu()
    
    print "start",rpy_start
    try:
        tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
            base_frame = 'base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
            rospy.signal_shutdown("tf Exception")

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
    rate = rospy.Rate(20)
    # position tranfrom from car to base
    (carPosition_inBaseFrame, carRotation_inBaseFrame) = get_odom()

    # get target position in car frame.
    (targetPosition_inCarFrame_x, targetPosition_inCarFrame_y, targetRotation_inCarFrame) = x,y,z

    # change target from car frame to base frame
    targetAngle_inCarFrame = atan2(targetPosition_inCarFrame_y, targetPosition_inCarFrame_x)
    targetAngle_inBaseFrame = piRange(targetAngle_inCarFrame + carRotation_inBaseFrame)

    targetDistance_inCarFrame = sqrt(pow((targetPosition_inCarFrame_x),2)+pow((targetPosition_inCarFrame_y),2))

    targetPosition_inBaseFrame_x = targetDistance_inCarFrame * cos(targetAngle_inBaseFrame) + carPosition_inBaseFrame.x
    targetPosition_inBaseFrame_y = targetDistance_inCarFrame * sin(targetAngle_inBaseFrame) + carPosition_inBaseFrame.y

    # vector_begin is the vector from target(in base frame) to car_start
    vector_begin_x = carPosition_inBaseFrame.x - targetPosition_inBaseFrame_x
    vector_begin_y = carPosition_inBaseFrame.y - targetPosition_inBaseFrame_y
    # vector_now is the vector from target(in base frame) to current car position
    vector_now_x, vector_now_y = vector_begin_x, vector_begin_y
    # their inner product, if position, smaller than 90 degree, else, larger.

    angle_kp_move = 5       # when move to a point. Should be large to change direction quickly
    angle_kp_rotate = 1     # when rotate to that point. Not too large.

    distnace_kp = 1
    angle_error = 0
    angle_error_accum = 0
    angle_ki = 0.5

    distance_error = targetDistance_inCarFrame
    move_cmd = Twist()
    cnt = 0
    start_time=time.time()

    # print "-Vector begin: ", vector_begin_x, vector_begin_y
    # print " == > In point control. Start to the point."
    while True:
        # print "--> distance_error:",distance_error
        if time.time()-start_time > 10:
            break
        angular_speed, linear_speed = 0, 0
        cnt += 1
        # print " --> While True cnt: --------------------------------->>>>", cnt
        (position, rotation) = get_odom()
        vector_now_x = position.x - targetPosition_inBaseFrame_x
        vector_now_y = position.y - targetPosition_inBaseFrame_y

        vector_product = vector_begin_x*vector_now_x + vector_begin_y*vector_now_y
        distance_error = sqrt(pow((targetPosition_inBaseFrame_x-position.x),2)+pow((targetPosition_inBaseFrame_y-position.y),2))

        if abs(targetPosition_inCarFrame_y) < 0.01 and targetPosition_inCarFrame_x < 0:
            # print "Move back"
            move_cmd.linear.x = -0.1
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            # print "Moc cmd:", move_cmd

            if vector_product <  0:
                move_cmd.linear.x, move_cmd.angular.z = 0, 0
                cmd_vel.publish(move_cmd)
                # print "--> Move back. Finished."
                # print "linear_speed,", linear_speed
                return 
            rate.sleep()
            # print "Cnt", cnt

        else:
            # print "distance error: ", distance_error, "  --------> ",cnt
            if distance_error < 0.02 or vector_product < 0:
                # print "vector product < 0"
                move_cmd.linear.x, move_cmd.angular.z = 0, 0
                cmd_vel.publish(move_cmd)
                if distance_error < 0.02:
                    print "--> Get to the target position."
                if vector_product < 0:
                    print "--> Over the target point, stopped"
                return 

            else:   # check angle
                angle_error = piRange(targetAngle_inBaseFrame - rotation)
                if (angle_error > -pi/4) and (angle_error < pi/4):  # move forward to position
                    # print "--> Move forward."

                    ''' if use accumulation
                    if abs(angle_error) < pi*5/180:
                        angle_error_accum += angle_error
                    if angle_error_accum > 0.001:
                        angle_error_accum = min(angle_error_accum, 0.5)
                    elif angle_error_accum < -0.001:
                        angle_error_accum = max(angle_error_accum, 0.5)
                    else:
                        pass
                    print "Angle error: ", angle_error, "accumulate: ",angle_error_accum
                    print " error contribute: ",angle_error * angle_kp_move, ", accum: ", angle_error_accum*angle_ki
                    angular_speed = angle_error * angle_kp_move + angle_error_accum * angle_ki
                    '''

                    linear_speed = distance_error * distnace_kp
                    angular_speed = angle_error * angle_kp_move


                elif (angle_error <= -pi/4 and angle_error >= -pi) or (angle_error >= pi/4 and angle_error <= pi):
                    # print "--> Rotate to face it"
                    angular_speed = angle_error * angle_kp_rotate

                    if angular_speed > 0:
                        angular_speed = max(angular_speed, 0.5)
                    else:
                        angular_speed = min(angular_speed, -0.5)

                    linear_speed = 0
                else: 
                    print "[Error] Enexpected angle..------------------------------Error"

            if angular_speed > 0.0001:
                move_cmd.angular.z = min(angular_speed, 2)
            elif angular_speed < -0.0001 :
                move_cmd.angular.z = max(angular_speed, -2)
            else:
                move_cmd.angular.z = 0

            if linear_speed > 0.0001:
                move_cmd.linear.x = min(linear_speed, 0.22)
            elif linear_speed < -0.0001:
                move_cmd.linear.x = max(linear_speed, -0.22)
            else:
                move_cmd.linear.x = 0
            # print "velocity: x, " , move_cmd.linear.x, "  --------> ",cnt
            cmd_vel.publish(move_cmd)
            rate.sleep()



def getkey():
    x, y, z = raw_input("| x | y | z |\n").split()
    if x == 's':
        shutdown()
    x, y, z = [float(x), float(y), float(z)]
    return x, y, z

def get_odom():
    global tf_listener,base_frame
    # global rev_odom,get_trans,get_odom_orientation
    # rev_odom=0
    # while rev_odom!=1:
    #     pass
    #     # print "wait odom" 
    # (_, _, get_odom_yaw) = tf.transformations.euler_from_quaternion([get_odom_orientation.x, get_odom_orientation.y, get_odom_orientation.z, get_odom_orientation.w])
    # return (get_trans,get_odom_yaw/2)

    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

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

def zhijiao(x,y,z):#0 heng 1 shu 2 finish
    zj_stage=0 #0 heng 1 shu 2 finish
    if zj_stage==0:
        if PointMove(0,y-0.01,0)==4:
            zj_stage=1
    if zj_stage==1:
        if PointMove(x,y,z)==4:
            zj_stage=2
    return zj_stage#0 heng 1 shu 2 finish
    

def piRange(angle):
    if angle > pi:
        angle = angle - 2*pi
    elif angle < -pi:
        angle = angle + 2*pi
    return  angle
def reset_odom():
    global pub_reset,global_pos,global_rot,position,rotation
    global_pos[0]=global_pos[0]+position.x
    global_pos[1]=global_pos[1]+position.y
    global_pos[2]=global_pos[2]+position.z
    global_rot=global_rot+rotation
    global_rot=global_rot%2
    while global_rot< -1:
        global_rot=global_rot+2
    while global_rot> 1:
        global_rot=global_rot-2
    pub_reset.publish(empty_msg)
    # while position.x>0.01 or position.y>0.01:
    #     (position, rotation) = get_odom()
    # new line by Dong yan
    print "Reset odom complete"

if __name__ == '__main__':

    move_start=0;
    rospy.init_node('mc3_main', anonymous=True)
    init()
    print 'hello'
    while not rospy.is_shutdown():
        (goal_x, goal_y, goal_z) = getkey()
        PointMove(goal_x, goal_y, goal_z)
        print "While once"