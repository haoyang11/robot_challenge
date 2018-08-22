#!/usr/bin/env python



import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from sensor_msgs.msg import Imu
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np

position=[]
rotation=0.0
get_imu_orientation=[]
get_odom_orientation=[]
def init():
    global cmd_vel,position,move_cmd,odom_frame,pub_reset
    # rospy.init_node('turtlebot3_pointop_key', anonymous=False)
    rospy.Subscriber("/odom", Odometry, odometry_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)

    position = Point()
    move_cmd = Twist()
    r = rospy.Rate(10)

def imu_callback(msg):
    global rev_imu,get_imu_orientation
    # print "i rev"
    if rev_imu==0:
        get_imu_orientation=msg.orientation
        rev_imu=1
def odometry_callback(msg):
    global rev_odom,get_trans,get_rotation,get_odom_orientation
    # print "i rev"
    if rev_odom==0:
        get_trans=msg.pose.pose.position
        get_rotation=msg.pose.pose.orientation.z*2
        get_odom_orientation=msg.pose.pose.orientation
        rev_odom=1

def get_imu():
    global rev_imu,get_imu_orientation
    rev_imu=0
    while rev_imu!=1:
        pass
        # print "wait odom"
    (getraw, getpitch, getyaw) = tf.transformations.euler_from_quaternion([get_imu_orientation.x, get_imu_orientation.y, get_imu_orientation.z, get_imu_orientation.w])
    getraw=getraw*180/pi 
    getpitch=getpitch*180/pi 
    getyaw=getyaw*180/pi 
    return getraw, getpitch, getyaw

    # try:
    #     (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
    #     rotation = euler_from_quaternion(rot)

    # except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    #     rospy.loginfo("TF Exception")
    #     return

    # return (Point(*trans), rotation[2])
def get_odom():
    global rev_odom,get_trans,get_rotation,get_odom_orientation
    rev_odom=0
    while rev_odom!=1:
        pass
        # print "wait odom"
    (getraw, getpitch, getyaw) = tf.transformations.euler_from_quaternion([get_imu_orientation.x, get_imu_orientation.y, get_imu_orientation.z, get_imu_orientation.w])
    # return (get_trans,get_rotation)
    return getraw, getpitch, getyaw

if __name__ == '__main__':
    move_start=0;
    rospy.init_node('mc3_main', anonymous=True)
    rate = rospy.Rate(10)
    init()
    print 'hello'
    while not rospy.is_shutdown():
        (raw, pitch, yaw)=get_imu()
        print "1:raw=",raw, "pitch=",pitch, "yaw=",yaw
        (raw, pitch, yaw)=get_odom()
        print "2:raw=",raw, "pitch=",pitch, "yaw=",yaw
        rate.sleep()
