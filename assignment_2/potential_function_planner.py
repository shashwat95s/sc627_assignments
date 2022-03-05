#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from math import atan2, pi, cos, sin
from helper import *
# import matplotlib.pyplot as plt


rospy.init_node('test', anonymous=True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

# read input file
f = open("catkin_ws/src/sc627_assignments/assignment_1/input.txt", "r")
l = f.readline()
start = [float(l[0]), float(l[2])]
l = f.readline()
goal = [float(l[0]), float(l[2])]
l = f.readline()
step = float(l)
l = f.readline()
l = f.readline()
p1 = [float(l[0]), float(l[2])]
l = f.readline()
p2 = [float(l[0]), float(l[2])]
l = f.readline()
p3 = [float(l[0]), float(l[2])]
l = f.readline()
l = f.readline()
p4 = [float(l[0]), float(l[2])]
l = f.readline()
p5 = [float(l[0]), float(l[2])]
l = f.readline()
p6 = [float(l[0]), float(l[2])]
f.close()
# start = [0, 0]
# goal = [5, 3]
# step = 0.1

# p1 = [1, 2]
# p2 = [1, 0]
# p3 = [3, 0]

# p4 = [2, 3]
# p5 = [4, 1]
# p6 = [5, 2]

P1 = [p1, p2, p3]
P2 = [p4, p5, p6]


k = 1
r = [[0, 0], [0, 0]]


# open output file
out = open("catkin_ws/src/sc627_assignments/assignment_2/output.txt", "a")

# setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0  # in radians (0 to 2pi)
wp = MoveXYGoal()


# Potential Functions Algorithm


while DistancePointToPoint(r[k], goal) > step:
    print("Start")
    [Ax, Ay] = attPotential(r[k], goal)
    [R1x, R1y] = repPotential(r[k], P1)
    [R2x, R2y] = repPotential(r[k], P2)

    [Vx, Vy, th] = normalise([Ax+R1x+R2x, Ay+R1y+R2y])

    newx = float(r[k][0]+step*Vx)
    newy = float(r[k][1]+step*Vy)
    # print(Ax, Ay)

    wp.pose_dest.x = newx
    wp.pose_dest.y = newy
    wp.pose_dest.theta = th
    client.send_goal(wp)
    print("sent message")

    client.wait_for_result()
    print("received message")
    result = client.get_result()
    r.append([result.pose_final.x, result.pose_final.y])
    out.write("\n% s," % result.pose_final.x)
    out.write("%s" % result.pose_final.y)
    k += 1
    if ((Ax**2)+(Ay**2)**0.5) < 0.2:
        break

print(r)

out.close()
