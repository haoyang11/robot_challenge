#!/usr/bin/env python

# Authors: Gilbert #


'''

Move to a forward point.
In three step:
    Step I: judge whether the angle is too large, if so, rotate to smaller angle
    Step II: move to the target point using distance and angle PID, stop at proper distance (within 5 cm)
    Step III: rotate to original orientation, using PID to control it in +-5 degree.


Problem:
    I: cannot move backward now
    II: maybe sometimes lose odom information, so the control time is delayed.

Restore:
    Add piRange

Dong yan. 2018.7.24


'''


import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np

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
mov_stage=0
stage_first=1
rev_odom=1
rev_imu=1

angle_kp = 5
angle_ki = 0
angle_kd = 2

distnace_kp = 1
distnace_ki = 0.01
distnace_kd = 0.5
    
move_end=0;
empty_msg=Empty()

global_pos=[0.0,0.0,0.0]
global_rot=0.0
position=[]
rotation=0.0
rpy_start=[0.0,0.0,0.0]
def init():
    global cmd_vel,position,move_cmd,odom_frame,pub_reset,rpy_start
    # rospy.init_node('turtlebot3_pointop_key', anonymous=False)
    rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.Subscriber("/odom", Odometry, odometry_callback)
    pub_reset = rospy.Publisher("/reset", Empty, queue_size=1)
    rospy.on_shutdown(shutdown)
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    position = Point()
    move_cmd = Twist()
    r = rospy.Rate(10)
    # tf_listener = tf.TransformListener()
    odom_frame = 'odom'
    rospy.sleep(1)
    reset_odom()
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
    global stage_first,move_end,mov_stage,cmd_vel,position,rotation,move_cmd,odom_frame,distance_error,goal_x, goal_y, goal_z,angle_error,angle_error_accum,angle_error_old,distance_error,distance_error_old,distance_error_accum,last_angle_error,last_distance_error
    print "mov_stage=",mov_stage
    if move_end==1:
        move_end=0
        mov_stage=0
    if mov_stage==0:
        mov_stage=1
        # cmd_vel=cmd_vel
        # position=position
        # move_cmd=move_cmd
        # r=r
        (position, rotation) = get_odom()
        # print "I get odom"

        # while 1:
        #     (position, rotation) = get_odom()
        #     print "position.x=", position.x, ", position.y=", position.y, " , rotation=", rotation

        (goal_x, goal_y, goal_z) = (x,y,z)
        

        last_rotation = 0
        linear_speed = 1
        angular_speed = 1

        angle_error = 0
        angle_error_accum = 0
        angle_error_old = 0
        distance_error = 0
        distance_error_old = 0
        distance_error_accum = 0

        last_angle_error = 0
        last_distance_error = 0

        

        if goal_z > 180 or goal_z < -180:
            print("you input worng z range.")
            shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        
        # distance = goal_distance

        # distance_error = distance

        to_face_front_flag = 1


        path_angle = atan2(goal_y - position.y, goal_x- position.x)
        angle_error = piRange(path_angle - rotation)
        distance_error = goal_distance

        # print "path_angle", path_angle, ", rotation: ", rotation, "angle_error", angle_error


        # print "Running insturction..."
        print angle_error

        # check forward or backward
        # while angle_error > pi/2 or angle_error < -pi/2:
        #     print "[Warning] Cannot move backward now."
        #     print "[Info]: angle_error= ",angle_error, ", ", angle_error*180/pi, " degree"
        #     cmd_vel.publish(Twist())
        #     # ros.sleep()


    # check angle error
    timeCnt = 0
    if mov_stage==1 :
        if angle_error > pi/4 or angle_error < -pi/4:
            if stage_first:
                stage_first=0
                # print "==> Step I: rotate to face front..."
            if angle_error > pi/8 or angle_error < -pi/8:
                path_angle = atan2(goal_y - position.y, goal_x- position.x)
                (position, rotation) = get_odom()
                angle_error_old = angle_error
                angle_error = piRange(path_angle - rotation)

                angle_error_accum = angle_error_accum + angle_error

                angle_speed = angle_error*angle_kp + (angle_error-angle_error_old)*angle_kd + angle_error_accum*angle_ki

                # print "angle_error=",angle_error,"Angle speed=",angle_speed
                if angle_speed > 0:
                    move_cmd.angular.z = min(angle_speed, 1.5)
                else:
                    move_cmd.angular.z = max(angle_speed, -1.5)

                move_cmd.linear.x = 0
                #print "angle speed: ", move_cmd.angular.z
                cmd_vel.publish(move_cmd)
                # r.sleep()
            else:
                # print "...................Now finished rotate......................"
                # print "==> Step II: move to target point..."
                mov_stage=2
        else:
            # print "...................Now finished rotate......................"
            # print "==> Step II: move to target point..."
            mov_stage=2
            stage_first=1

    if mov_stage==2 :
        if distance_error > 0.02:
            #print "Times: ",timeCnt
            # if timeCnt > 200:
            #     print "**********  Time Out ***************"
            #     break

            (position, rotation) = get_odom()
            path_angle = atan2(goal_y - position.y, goal_x- position.x)

            distance_error = sqrt(pow((goal_x - position.x), 2) + pow((goal_y - position.y), 2))
            angle_error = piRange(path_angle - rotation)
            
            linear_speed = distance_error * distnace_kp
            angular_speed = angle_error * angle_kp
            #print "Position: ", position

            #print "Error: dis: ",distance_error,", ang: ",angle_error,",   Output: dis:", linear_speed,", ang: ",angular_speed


            if linear_speed > 0:
                move_cmd.linear.x = min(linear_speed, 0.22)
            else :
                move_cmd.linear.x = max(linear_speed, -0.22)

            if angular_speed > 0:
                move_cmd.angular.z = min(angular_speed, 1)
            else :
                move_cmd.angular.z = max(angular_speed, -1)

            last_rotation = rotation
            cmd_vel.publish(move_cmd)
            # r.sleep()
        else:
            # print "...................Now finished linear......................"
            # print "==> Step III: rotate back to orientation..."
            (position, rotation) = get_odom()
            angle_error = piRange(goal_z - rotation)
            # print "rotation: ",rotation, "goal_z: ",goal_z, "error: ", angle_error, "  threshold: ", pi/180*10
            mov_stage=3
    
    if mov_stage==3 : 
        #while angle_error > pi/180*5 or angle_error < -pi/180*5:
        if angle_error > pi/180*10 or angle_error < -pi/180*10:
            (position, rotation) = get_odom()

            angle_error_old = angle_error
            angle_error_accum = angle_error_accum + angle_error
            if angle_error_accum > 0.5 :
                angle_error_accum = 0.5 
            if angle_error_accum < -0.5 :
                angle_error_accum = 0.5

            angle_error = piRange(goal_z - rotation)

            angular_speed = angle_error * angle_kp + (angle_error-angle_error_old)*angle_kd + angle_error_accum*angle_ki

            if angular_speed > 0:
                move_cmd.angular.z = min(angular_speed, 1)
            else :
                move_cmd.angular.z = max(angular_speed, -1)
            move_cmd.linear.x = 0
            cmd_vel.publish(move_cmd)
            # print "rotation: ",rotation,  "error: ", angle_error, "anglular_speed: ", angular_speed
            # r.sleep()
        else:
            # print '---------------------------------------------'
            # print '             stop move part.'
            # print '---------------------------------------------'
            rospy.loginfo("Stopping the robot...")
            cmd_vel.publish(Twist())
            (position, rotation) = get_odom()
            print position, rotation
            # reset_odom()
            mov_stage=4
            move_end=1
    return mov_stage

def getkey():
    x, y, z = raw_input("| x | y | z |\n").split()
    if x == 's':
        shutdown()
    x, y, z = [float(x), float(y), float(z)]
    return x, y, z

def get_odom():
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

    # return (Point(*trans), rotation[2])
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
    rate = rospy.Rate(10)
    init()
    print 'hello'
    while not rospy.is_shutdown():
        if move_start==0:
            print(msg)
            (goal_x, goal_y, goal_z) = getkey()
            move_start=1
        if move_start==1:
            if PointMove(goal_x, goal_y, goal_z)==4:
            # if zhijiao(goal_x, goal_y, goal_z)==2:
                move_start=0
                reset_odom()
                # print "global_rot=",global_rot
                # mov_stage=0;
        rate.sleep()
