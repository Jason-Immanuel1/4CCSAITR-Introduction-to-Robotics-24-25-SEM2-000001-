#!/usr/bin/env python3

import rospy
import math
import angles
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


goal = (10, 5)

rospy.init_node('goto_node')
vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)


def receive_position(current: Pose):
    command = Twist()

    diff = (goal[0] - current.x, goal[1] - current.y)
    goal_orientation = math.atan2(diff[1], diff[0])

    angle_diff = angles.shortest_angular_distance(current.theta, goal_orientation)

    if angle_diff < -0.087 or angle_diff > 0.087:
        command.angular.z = -1
    elif math.hypot(*diff) > 0.3:
        command.linear.x = 1

    vel.publish(command)


rospy.Subscriber('/turtle1/pose', Pose, receive_position)
rospy.spin()
