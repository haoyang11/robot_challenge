#!/usr/bin/env python
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

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


class GotoPoint():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

    def PointMove(self,x,y,z):
        cmd_vel=self.cmd_vel
        position=self.position
        move_cmd=self.move_cmd
        r=self.r
        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        # (goal_x, goal_y, goal_z) = self.getkey()
        (goal_x, goal_y, goal_z) = (x,y,z)

        angle_error = 0
        angle_error_accum = 0
        angle_error_old = 0
        distance_error = 0
        distance_error_old = 0
        distance_error_accum = 0

        last_angle_error = 0
        last_distance_error = 0

        angle_kp = 5
        angle_ki = 0
        angle_kd = 2

        distnace_kp = 1
        distnace_ki = 0.01
        distance_kd = 0.5

        if goal_z > 180 or goal_z < -180:
            print("you input worng z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        distance_error = distance

        to_face_front_flag = 1


        path_angle = atan2(goal_y - position.y, goal_x- position.x)
        angle_error = self.piRange(path_angle - rotation)
        distance_error = goal_distance

        print "path_angle", path_angle, ", rotation: ", rotation, "angle_error", angle_error


        print "Running insturction..."
        print angle_error

        # check forward or backward
        while angle_error > pi/2 or angle_error < -pi/2:
            print "[Warning] Cannot move backward now."
            print "[Info]: angle_error= ",angle_error, ", ", angle_error*180/pi, " degree"
            self.cmd_vel.publish(Twist())
            rospy.sleep()


        # check angle error
        timeCnt = 0
        if angle_error > pi/4 or angle_error < -pi/4:
            print "==> Step I: rotate to face front..."

            while angle_error > pi/8 or angle_error < -pi/8:
                path_angle = atan2(goal_y - position.y, goal_x- position.x)
                (position, rotation) = self.get_odom()
                angle_error_old = angle_error
                angle_error = self.piRange(path_angle - rotation)

                angle_error_accum = angle_error_accum + angle_error

                angle_speed = angle_error*angle_kp + (angle_error-angle_error_old)*angle_kd + angle_error_accum*angle_ki

                print "Angle speed: ",angle_speed
                if angle_speed > 0:
                    move_cmd.angular.z = min(angle_speed, 1.5)
                else:
                    move_cmd.angular.z = max(angle_speed, -1.5)

                move_cmd.linear.x = 0
                #print "angle speed: ", move_cmd.angular.z
                self.cmd_vel.publish(move_cmd)
                r.sleep()
                
                
            print "...................Now finished rotate......................"

        
        print "==> Step II: move to target point..."
        while distance_error > 0.05:

            #print "Times: ",timeCnt
            if timeCnt > 200:
                print "**********  Time Out ***************"
                break

            (position, rotation) = self.get_odom()
            path_angle = atan2(goal_y - position.y, goal_x- position.x)

            distance_error = sqrt(pow((goal_x - position.x), 2) + pow((goal_y - position.y), 2))
            angle_error = self.piRange(path_angle - rotation)
            
            linear_speed = distance_error * distnace_kp
            angular_speed = angle_error * angle_kp

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
            self.cmd_vel.publish(move_cmd)
            r.sleep()
        
        (position, rotation) = self.get_odom()

        angle_error = self.piRange(goal_z - rotation)
        print "==> Step III: rotate back to orientation..."
        print "rotation: ",rotation, "goal_z: ",goal_z, "error: ", angle_error, "  threshold: ", pi/180*10
        #while angle_error > pi/180*5 or angle_error < -pi/180*5:
        while angle_error > pi/180*10 or angle_error < -pi/180*10:
            (position, rotation) = self.get_odom()

            angle_error_old = angle_error
            angle_error_accum = angle_error_accum + angle_error
            if angle_error_accum > 0.5 :
                angle_error_accum = 0.5 
            if angle_error_accum < -0.5 :
                angle_error_accum = 0.5

            angle_error = self.piRange(goal_z - rotation)

            angular_speed = angle_error * angle_kp + (angle_error-angle_error_old)*angle_kd + angle_error_accum*angle_ki

            if angular_speed > 0:
                move_cmd.angular.z = min(angular_speed, 1)
            else :
                move_cmd.angular.z = max(angular_speed, -1)
            move_cmd.linear.x = 0
            self.cmd_vel.publish(move_cmd)
            print "rotation: ",rotation,  "error: ", angle_error, "anglular_speed: ", angular_speed
            r.sleep()


        print '---------------------------------------------'
        print '             stop move part.'
        print '---------------------------------------------'
        


        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def getkey(self):
        x, y, z = raw_input("| x | y | z |\n").split()
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def piRange(self,angle):
        if angle > pi:
            angle = angle - 2*pi
        elif angle < -pi:
            angle = angle + 2*pi
        return  angle


if __name__ == '__main__':
    mymove=GotoPoint()
    # rospy.init_node('mc3_main', anonymous=True)
    rate = rospy.Rate(10)
    print 'hello'
    while not rospy.is_shutdown():
        print(msg)
        (goal_x, goal_y, goal_z) = mymove.getkey()
        # mymove.PointMove(0.5,0.1,0)
        mymove.PointMove(goal_x, goal_y, goal_z)
        (position, rotation) = mymove.get_odom()
        print position, rotation
        rate.sleep()
