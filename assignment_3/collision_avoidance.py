#!/usr/bin/env python

from re import A
import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from math import cos, sin
import numpy as np

ANG_MAX = math.pi/18
VEL_MAX = 0.15


class Assign3:

    def __init__(self):
        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size=10)
        self.r = rospy.Rate(30)
        self.obs_data = ObsData()
        self.odom = Odometry()

    def velocity_convert(self, x, y, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 1  # modify if necessary

        ang = math.atan2(vel_y, vel_x)
        if ang < 0:
            ang += 2 * math.pi

        ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

        v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x **
                    2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang

    def callback_obs(self, data):
        '''
        Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
        '''
        # global Obstacle_data
        # for i, j in zip(data.obstacles, range(len(data))):
        #     Obstacle_data[j][0] = i.obs
        #     Obstacle_data[j][1] = i.pose_x
        #     Obstacle_data[j][2] = i.pose_y
        #     Obstacle_data[j][3] = i.vel_x
        #     Obstacle_data[j][4] = i.vel_y
        # return Obstacle_data
        self.obs_data = data.obstacles
        print("length")
        print(len(data))

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        self.odom = data
        # print(data)


if __name__ == '__main__':
    rospy.init_node('assign3_skeleton', anonymous=True)
    s = Assign3()
    rospy.Subscriber('/obs_data', ObsData, s.callback_obs)  # topic name fixed
    rospy.Subscriber('/bot_1/odom', Odometry,
                     s.callback_odom)  # topic name fixed
    a = 1
while a < 5:  # replace with destination reached?
    # access obs_data and odom
    # Obstacle_data = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
    # for i, j in zip(s.obs_data, range(len(s.obs_data))):
    #     Obstacle_data[j][0] = i.obs
    #     Obstacle_data[j][1] = i.pose_x
    #     Obstacle_data[j][2] = i.pose_y
    #     Obstacle_data[j][3] = i.vel_x
    #     Obstacle_data[j][4] = i.vel_y

    print('Obs Data \n', s.obs_data)
    print('---')
    # print("length")
    # print(len(s.obs_data.obstacles))
    print('Odometry\n', s.odom)
    print('***')
    # d=0.15
    # x = s.odom.pose.pose.position.x
    # y = s.odom.pose.pose.position.y
    # ori = s.odom.pose.pose.orientation
    # (roll, pitch, th) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    # vl=s.odom.twist.twist.linear.x

    # vy=s.odom.twist.twist.linear.y

    # x1
    # y1
    # vx1
    # vy1

    # calculate collision cone below
    # D= ((x-x1)**2+(y-y1)**2)**0.5
    # alpha= np.arcsin(d/D)
    # theta1 = atan2(y1-y,x1-x)

    # calculate v_x, v_y as per either TG, MV, or ST strategy
    # v_des_l=0.15
    #  beta= atan2(v_des_l*sin(th)-vy1,v_des_l*cos(th)-vx1)
    # while beta<theta1+alpha and beta > theta-alpha:
    #   v_des_l=v_des_l-0.01
    #   beta= atan2(v_des_l*sin(th)-vy1,v_des_l*cos(th)-vx1)
    #   if v_des_l<0:
    #     break

    # Make sure your velocity vector is feasible (magnitude and direction)

    # convert velocity vector to linear and angular velocties using velocity_convert function given above

    # publish the velocities below
    vel_msg = Twist()
    # vel_msg.linear.x = v_lin
    # vel_msg.angular.z = v_ang
    s.pub_vel.publish(vel_msg)

    # store robot path with time stamps (data available in odom topic)
    a = a+1
    s.r.sleep()
