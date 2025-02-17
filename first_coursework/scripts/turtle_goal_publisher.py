#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Point
from turtlesim.msg import Pose

class TurtleGoalPublisher:
    def __init__(self):
        rospy.init_node('turtle_goal_publisher', anonymous=True)
        
        self.goal_pub = rospy.Publisher('/goal', Point, queue_size=10)
        
        self.turtle_pose = None
        self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        self.rate = rospy.Rate(10)  

    def pose_callback(self, msg):
        self.turtle_pose = msg

    def calculate_euclidean_distance(self, goal_x, goal_y):
        if self.turtle_pose is None:
            return None
        return math.sqrt((goal_x - self.turtle_pose.x)**2 + (goal_y - self.turtle_pose.y)**2)
    
    def normalize_coordinates(self, a):
        return (a * 2) + 1

    def publish_goal(self, x, y):
        while not rospy.is_shutdown():
            try:

                goal_x = self.normalize_coordinates(x)
                goal_y = self.normalize_coordinates(y)

                goal = Point()
                goal.x = goal_x
                goal.y = goal_y
                goal.z = 0.0

                self.goal_pub.publish(goal)

                distance = self.calculate_euclidean_distance(goal.x, goal.y)
                if distance is not None:
                    rospy.loginfo(f"Distance to goal ({goal_x}, {goal_y}): {distance}")

                    if distance < 0.1:
                        rospy.loginfo(f"Goal ({goal_x}, {goal_y}) reached!")
                        break

            except Exception as e:
                rospy.logwarn(f"Error: {e}")

            self.rate.sleep()

    def publish_multiple_goals(self, goals):
        for goal in goals:
            self.publish_goal(goal[0], goal[1])
            rospy.loginfo(f"Moving to next goal: {goal}")

if __name__ == "__main__":
    try:
        turtle_goal_publisher = TurtleGoalPublisher()

        goals = [(0.0, 0.0), (1.0, 0.0), (0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0), (1.0, 1.0), (0.0, 2.0), (0.0, 1.0), (1.0, 2.0), (0.0, 2.0), (1.0, 3.0), (0.0, 3.0), (0.0, 4.0), (0.0, 3.0), (1.0, 4.0), (0.0, 4.0), (1.0, 3.0), (2.0, 4.0), (1.0, 4.0), (2.0, 3.0), (1.0, 3.0), (2.0, 4.0), (3.0, 3.0), (2.0, 3.0), (3.0, 4.0), (2.0, 4.0), (3.0, 3.0), (4.0, 4.0), (3.0, 4.0), (3.0, 2.0), (2.0, 2.0), (1.0, 2.0), (2.0, 3.0), (3.0, 2.0), (2.0, 1.0), (1.0, 1.0), (2.0, 0.0), (1.0, 0.0), (2.0, 1.0), (3.0, 0.0), (2.0, 0.0), (3.0, 1.0), (2.0, 1.0), (3.0, 2.0), (4.0, 1.0), (3.0, 1.0), (4.0, 0.0), (3.0, 0.0), (4.0, 1.0), (4.0, 2.0), (3.0, 2.0), (4.0, 1.0), (3.0, 0.0), (4.0, 0.0), (2.0, 2.0), (3.0, 1.0), (3.0, 3.0), (4.0, 2.0), (3.0, 1.0), (2.0, 2.0), (3.0, 4.0), (1.0, 3.0), (0.0, 2.0), (1.0, 4.0), (2.0, 3.0), (1.0, 2.0), (2.0, 2.0), (1.0, 1.0), (0.0, 3.0), (1.0, 0.0), (1.0, 2.0), (2.0, 1.0), (2.0, 0.0), (0.0, 1.0)]

        turtle_goal_publisher.publish_multiple_goals(goals)
    except rospy.ROSInterruptException:
        pass

