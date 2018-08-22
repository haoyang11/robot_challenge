#!/usr/bin/env python

'''
Motion control on pi, basic codes like driver.

1. Use subscirber to receive control command from pi
2. Control move by PointMove, and rotate by rotateTo()
3. After finishing control, give a state flag to pi.
4. After pi receving the finish flag, it begin to run the next-line codes
5. The paramter of PointMove and rotateTo2 is set by Dongyan at 8.8.


Update time: 2018.8.8

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

point_rev=False
rotate_rev=False
point_goal=[]
rotate_goal=[]

def point_control_callback(msg):
    global point_rev,point_goal
    point_goal=msg.data
    print "point_rev=",point_goal
    point_rev=True

def rotate_control_callback(msg):
    global rotate_rev,rotate_goal
    rotate_goal=msg.data
    print "rotate_rev=",rotate_goal
    rotate_rev=True
    
def reset_callback(msg):
    reset()
    
def reset():
    global cmd_vel,position,move_cmd,odom_frame,pub_reset,rpy_start,tf_listener,base_frame
    global pub_rotate_state,pub_point_state

    position = Point()
    move_cmd = Twist()
    odom_frame = 'odom'
    rospy.sleep(1)
    rpy_start=get_imu(True)
    reset_odom()
    
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

def init():
    global cmd_vel,position,move_cmd,odom_frame,pub_reset,rpy_start,tf_listener,base_frame
    global pub_rotate_state,pub_point_state
    rospy.Subscriber("/tb"+tb_name+"/imu", Imu, imu_callback)
    rospy.Subscriber("/tb"+tb_name+"/odom", Odometry, odometry_callback)
    pub_reset = rospy.Publisher("/tb"+tb_name+"/reset", Empty, queue_size=1)
    rospy.on_shutdown(shutdown)
    cmd_vel = rospy.Publisher("/tb"+tb_name+'/cmd_vel', Twist, queue_size=5)
    
    rospy.Subscriber("/tb"+tb_name+"/point_control", Float64MultiArray, point_control_callback)
    pub_point_state=rospy.Publisher("/tb"+tb_name+"/point_state", Bool, queue_size=1)
    rospy.Subscriber("/tb"+tb_name+"/rotate_control", Float64MultiArray, rotate_control_callback)
    pub_rotate_state=rospy.Publisher("/tb"+tb_name+"/rotate_state", Bool, queue_size=1)
    rospy.Subscriber("/tb"+tb_name+"/motion_control_reset", Empty, reset_callback)

    position = Point()
    move_cmd = Twist()
    r = rospy.Rate(100)
    tf_listener = tf.TransformListener()
    odom_frame = 'odom'
    rospy.sleep(1)
    rpy_start=get_imu(True)
    reset_odom()
    
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

def PointMove(x,y,z=0):
    print "--> Move to point (", x, ", ",  y, ", ",  z, ")."
    t = time.time()
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

        distance_error_old = distance_error

        distance_error = sqrt(pow((targetPosition_inBaseFrame_x-position.x),2)+pow((targetPosition_inBaseFrame_y-position.y),2))

        if abs(targetPosition_inCarFrame_y) < 0.05 and targetPosition_inCarFrame_x < 0:
            # print "Move back"
            move_cmd.linear.x = -0.2
            # move_cmd.linear.x = targetPosition_inCarFrame_y * 1
            move_cmd.angular.z = 0
            cmd_vel.publish(move_cmd)
            # print "Moc cmd:", move_cmd

            if vector_product <  0:
                move_cmd.linear.x, move_cmd.angular.z = 0, 0
                cmd_vel.publish(move_cmd)
                # print "--> Move back. Finished."
                # print "linear_speed,", linear_speed
		print "-->          Finished moving. Time use:", (int)((time.time()-t)*1000), "ms"
                return 
            # rate.sleep()
            # print "Cnt", cnt

        else:
            # print "distance error: ", distance_error, "  --------> ",cnt
            if distance_error < 0.05 or vector_product < 0:
                # print "vector product < 0"
                move_cmd.linear.x, move_cmd.angular.z = 0, 0
                cmd_vel.publish(move_cmd)
                if distance_error < 0.05:
                    # print "--> Get to the target position."
		    pass
                if vector_product < 0:
                    # print "--> Over the target point, stopped"
		    pass
		print "-->          Finished moving. Time use:", (int)((time.time()-t)*1000), "ms"
                return 

            else:   # check angle
                angle_error_old = angle_error

                angle_error = piRange(targetAngle_inBaseFrame - rotation)
                if (angle_error > -pi/4) and (angle_error < pi/4):  # move forward to position

                    linear_speed = distance_error * 2 + (distance_error - distance_error_old) * 0
                    angular_speed = angle_error * 5 + (angle_error - angle_error_old) * 0


                elif (angle_error <= -pi/4 and angle_error >= -pi) or (angle_error >= pi/4 and angle_error <= pi):
                    # print "--> Rotate to face it"
                    angular_speed = angle_error * 1 + (angle_error - angle_error_old) * 0

                    if angular_speed > 0:
                        angular_speed = max(angular_speed, 2)
                    else:
                        angular_speed = min(angular_speed, -2)

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
            # rate.sleep()



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
def get_imu(absolute=False):
    global rev_imu,get_imu_orientation,rpy_start
    rev_imu=0
    while rev_imu!=1:
        pass
        # print "wait odom"
    get_rpy = list(tf.transformations.euler_from_quaternion([get_imu_orientation.x, get_imu_orientation.y, get_imu_orientation.z, get_imu_orientation.w]))
    for i in range(3):
        if absolute==True:
            rpy_start[i]=0
        get_rpy[i]=(get_rpy[i])*180/pi -rpy_start[i]
        if(get_rpy[i]>180):
            get_rpy[i]=get_rpy[i]-360
        if(get_rpy[i]<-180):
            get_rpy[i]=get_rpy[i]+360
    print"get_rpy=",get_rpy
    return get_rpy

def shutdown():
    global cmd_vel
    cmd_vel.publish(Twist())
    rospy.sleep(1)

def rotateTo2(angle):     # rotate to a certain angle, with default speed 0.4
    print " --> Rotate to angle: ", angle, " degree."
    t = time.time()
    rate = rospy.Rate(1000)
    # global rate
    rpy=get_imu()
    angle_error = angle - rpy[2]
    if(angle_error > 180):
        angle_error -= 360
    elif(angle_error < -180):
        # print "angle_error,", angle_error
        angle_error += 360
        # print "angle_error,", angle_error

    # print "<rotateTo> Input angle:", angle, "current angle: ",rpy[2], "angle error:", angle_error
    # print "angle_error: ", angle_error,"angle_now: ", rpy[2]

    angle_error_old = 0

    while abs(angle_error) > 5:

        rpy=get_imu()
        # print "angle_error: ", angle_error,"angle_now: ", rpy[2]
        angle_error_old = angle_error   # pd
        angle_error = angle - rpy[2]
        if(angle_error > 180):
            angle_error -= 360
        elif(angle_error < -180):
            angle_error += 360

	#print "current:", rpy[2], "angle  error", angle_error

        speed = angle_error * 0.05 + (angle_error - angle_error_old) * 0.0

        # speed = max(angle_error*0.02, 0.4)  #lowerspped, 0.4
        if speed > 0:
            speed = min(speed, 2.5) # upperspeed, 2
        else:
            speed = max(speed, -2.5)
        vel_control(0, speed)    # low speed to rotate to front
        # print "<rotateTo> Stopped: ",rpy[2], "angle error:", angle_error


        # print "angle_error", angle_error
        rate.sleep()
    #print "<rotateTo> Rotate finished at angle: ", rpy[2]

    print "-->          Finished rotating. Time use:", (int)((time.time()-t)*1000), "ms"
    vel_control(0, 0)   

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

    ####rotate######
    angle=float(raw_input("angle\n"))
    rotateTo2(angle)



# point_rev,rotate_rev,point_goal,rotate_goal,pub_rotate_state,pub_point_state

if __name__ == '__main__':
    global point_rev,rotate_rev,point_goal,rotate_goal
    global pub_rotate_state,pub_point_state
    rospy.init_node("tb"+tb_name+'motion_control', anonymous=True)
    init()
    print " ------------------------------------------------------" 
    print '    Pi control driver is ready. Waiting for command' 
    print " ------------------------------------------------------" 
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        # test()
        if rotate_rev:
            rotate_rev=False
            pub_rotate_state.publish(False)
            rotateTo2(rotate_goal[0])
            pub_rotate_state.publish(True)
        if point_rev:
            point_rev=False
            pub_point_state.publish(False)
            PointMove(point_goal[0],point_goal[1],point_goal[2])
            pub_point_state.publish(True)
        rate.sleep()
