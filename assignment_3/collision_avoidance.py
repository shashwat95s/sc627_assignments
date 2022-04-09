#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from math import sin, cos, atan2, pi
import numpy as np
from tf.transformations import euler_from_quaternion

ANG_MAX = math.pi/18
VEL_MAX = 0.15
Obstacle_data = [[0, 0, 0, 0, 0], [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]]
od = [0, 0, 0, 0, 0]


def check_collision(can_vel, bot_pose, obs_pose):
    d = 0.2
    D = max(((bot_pose[0]-obs_pose[1])**2 +
            (bot_pose[1]-obs_pose[2])**2)**0.5, 0.3)

    alpha = np.arcsin(d/D)
    theta1 = atan2(-(bot_pose[1]-obs_pose[2]), -(bot_pose[0]-obs_pose[1]))

    beta = atan2(can_vel[1]-obs_pose[4], can_vel[0]-obs_pose[3])

    if beta < theta1+alpha and beta > theta1-alpha:
        return 1
    else:
        return 0


def find_vector(bot_pose, all_obs, goal):
    flag = 0
    best = [0, 0]  # [maginitude, angle]
    mag = []
    ang = []
    goal_ang = atan2(goal[1]-bot_pose[1], goal[0]-bot_pose[0])
    for i in range(15):
        mag.append(0.01+i*0.01)

    for i in range(21):
        ang.append((-10+i)*pi/180)

    if bot_pose[0] > 4:
        r = 4
    else:
        r = len(mag)
    for i in range(r):
        for j in range(len(ang)):
            can_x = mag[i]*cos(ang[j]+bot_pose[2])
            can_y = mag[i]*sin(ang[j]+bot_pose[2])

            col = 0
            for k in range(3):
                col = col+check_collision([can_x, can_y], bot_pose, all_obs[k])
            if col == 0:
                if flag == 0:
                    best = [mag[i], ang[j]]
                    flag = 1
                elif abs((ang[j]+bot_pose[2])-goal[1]) <= abs((best[1]+bot_pose[2])-goal[1]):
                    best = [mag[i], ang[j]]
                    # print("ang_cond")
                elif ang[j] == best[1] and mag[i] > best[0]:
                    # print("mag condition")
                    best = [mag[i], ang[j]]
            # print("best in loop")
            # print([mag[i], ang[j]])
            # print(best)

    v_x = best[0]*cos(best[1]+bot_pose[2])
    v_y = best[0]*sin(best[1]+bot_pose[2])

    return [v_x, v_y]


def velocity_convert(x, y, theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''
    global od
    gain_ang = 1.4  # modify if necessary
    if od[0] > 4:
        gain_ang = 2
    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi

    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x **
                2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err

    return v_lin, v_ang


def callback_obs(data):
    '''
    Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
    '''
    global Obstacle_data

    for i, j in zip(data.obstacles, range(len(data.obstacles))):
        Obstacle_data[j][0] = i.obs
        Obstacle_data[j][1] = i.pose_x
        Obstacle_data[j][2] = i.pose_y
        Obstacle_data[j][3] = i.vel_x
        Obstacle_data[j][4] = i.vel_y
    return Obstacle_data
    # print(data)
    # pass


def callback_odom(data):
    '''
    Get robot data
    '''
    global od
    od[0] = data.pose.pose.position.x
    od[1] = data.pose.pose.position.y
    ori = data.pose.pose.orientation
    (roll, pitch, th) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
    od[2] = th  # data.odom.twist.twist.linear.x
    od[3] = data.twist.twist.linear.x
    od[4] = data.twist.twist.angular.z
    # print(data)
    pass


rospy.init_node('assign3_skeleton', anonymous=True)
rospy.Subscriber('/obs_data', ObsData, callback_obs)  # topic name fixed
rospy.Subscriber('/bot_1/odom', Odometry, callback_odom)  # topic name fixed

pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size=10)
r = rospy.Rate(30)
goal = [5, 0]
x = 0
y = 0
fl = 0

xout = open(
    "/home/user/catkin_ws/src/sc627_assignments/assignment_3/xout.txt", "a")
yout = open(
    "/home/user/catkin_ws/src/sc627_assignments/assignment_3/yout.txt", "a")
while ((x-goal[0])**2+(y-goal[1])**2)**0.5 > 0.4:  # replace with destination reached?
    # calculate collision cone below
    x = od[0]
    y = od[1]
    th = od[2]
    # calculate v_x, v_y as per either TG, MV, or ST strategy
    [v_x, v_y] = find_vector(od, Obstacle_data, goal)

    # convert velocity vector to linear and angular velocties using velocity_convert function given above
    [v_lin, v_ang] = velocity_convert(x, y, th % (2*pi), v_x, v_y)
    # publish the velocities below

    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    pub_vel.publish(vel_msg)

    # store robot path with time stamps (data available in odom topic)
    xout.write(str(x))
    xout.write("\n")
    yout.write(str(y))
    yout.write("\n")

    r.sleep()
vel_msg.linear.x = 0
vel_msg.angular.z = 0
pub_vel.publish(vel_msg)
xout.close()
yout.close()
