#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
from math import pi

ANG_MAX = math.pi/18
VEL_MAX = 0.15


class Assign4:

    def __init__(self):
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(30)
        self.left_odom = Odometry()
        self.right_odom = Odometry()
        self.odom = Odometry()

    def velocity_convert(self, x, y, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 0.1  # modify if necessary

        ang = math.atan2(vel_y, vel_x)
        ang += 2 * math.pi

        ang_err = ang - theta
        if ang_err < -pi:
            ang_err += 2 * math.pi
        elif ang_err > pi:
            ang_err -= 2*pi

        if ang_err > pi/2:
            ang_err = max(ang_err, pi-ANG_MAX)
        elif ang_err < -pi/2:
            ang_err = min(ang_err, -pi+ANG_MAX)
        else:
            ang_err = min(max(ang_err, -ANG_MAX), ANG_MAX)

        v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x **
                    2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
        v_ang = gain_ang * math.sin(ang_err)
        return v_lin, v_ang

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.odom = data

    def callback_left_odom(self, data):
        '''
        Get left robot data
        '''

        self.left_odom = data

    def callback_right_odom(self, data):
        '''
        Get right robot data
        '''
        # print(data)
        self.right_odom = data


if __name__ == '__main__':
    rospy.init_node('assign4_skeleton', anonymous=True)
    s = Assign4()
    rospy.Subscriber('/odom', Odometry, s.callback_odom)  # topic name fixed
    rospy.Subscriber('/left_odom', Odometry,
                     s.callback_left_odom)  # topic name fixed
    rospy.Subscriber('/right_odom', Odometry,
                     s.callback_right_odom)  # topic name fixed

name = str(rospy.get_namespace())
if name == "/bot_2/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/out_2.txt", "a")
elif name == "/bot_3/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/out_3.txt", "a")
elif name == "/bot_4/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/out_4.txt", "a")
elif name == "/bot_5/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/out_5.txt", "a")
elif name == "/bot_6/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/out_6.txt", "a")
elif name == "/bot_7/":
    out = open(
        "/home/user/catkin_ws/src/sc627_assignments/assignment_4/out_7.txt", "a")

k = 1
i = 1
v_x = 1
lv = 1
rv = 1

while i < 200 or abs(v_x) > 0.001 or abs(lv) > 0.001 or abs(rv) > 0.001:

    x = s.odom.pose.pose.position.x
    y = s.odom.pose.pose.position.y
    ori = s.odom.pose.pose.orientation
    (roll, pitch, th) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    rx = s.right_odom.pose.pose.position.x
    ry = s.right_odom.pose.pose.position.y
    rv = s.right_odom.twist.twist.linear.x
    lx = s.left_odom.pose.pose.position.x
    ly = s.left_odom.pose.pose.position.y
    lv = s.left_odom.twist.twist.linear.x

    # calculate v_x, v_y as per the balancing strategy
    v_x = k*((rx-x)+(lx-x))
    v_y = k*((ry-y)+(ly-y))

    # the same function takes care of feasible velocities and also converts velocities
    [v_lin, v_ang] = s.velocity_convert(x, y, th % (2*pi), v_x, v_y)

    # publish the velocities below
    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    s.pub_vel.publish(vel_msg)

    # store robot path with time stamps (data available in odom topic)
    out.write(str(i))
    out.write("\n")
    out.write(str(x))
    out.write("\n")
    out.write(str(y))
    out.write("\n")

    i = i+1
    s.r.sleep()

vel_msg.linear.x = 0
vel_msg.angular.z = 0
s.pub_vel.publish(vel_msg)
out.close()
