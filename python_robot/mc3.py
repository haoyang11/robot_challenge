import point_control
import rospy


if __name__ == '__main__':
    mymove=point_control.GotoPoint()
    print 'hello'
    while not rospy.is_shutdown():
        print(point_control.msg)
        (goal_x, goal_y, goal_z) = mymove.getkey()
        # mymove.PointMove(0.5,0.1,0)
        mymove.PointMove(goal_x, goal_y, goal_z)
        (position, rotation) = mymove.get_odom()
        print position, rotation
