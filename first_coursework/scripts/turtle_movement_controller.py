#!/usr/bin/env python3

import rospy
import math
import angles
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point

class TurtleMovementController:

    def __init__(self):
        rospy.init_node('turtle_controller')
        rospy.Subscriber('/turtle1/pose', Pose, self.recieve_postion)
        rospy.Subscriber('/goal', Point, self.recieve_goal)

        self.vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.current_pose : Pose = None
        self.goal : Point = None

    def recieve_postion(self, current : Pose):
        self.current_pose = current

    def recieve_goal(self, goal : Point):
        self.goal = goal
        
    def turtle_control_loop(self):

        while not rospy.is_shutdown():
            if self.current_pose is not None and self.goal is not None:
                command = Twist()

                x_difference = self.goal.x - self.current_pose.x
                y_difference = self.goal.y - self.current_pose.y
                goal_orientation = math.atan2(y_difference, x_difference)

                angle_diff = angles.shortest_angular_distance(self.current_pose.theta, goal_orientation)
                distance = math.hypot(x_difference, y_difference)

                if abs(angle_diff) > 0.01:  
                    command.angular.z = 0.1 * angle_diff  
                elif distance > 0.1:  
                    command.linear.x = 0.1 * distance  
                else:
                    rospy.loginfo(f"Goal reached at position: ({self.goal.x}, {self.goal.y})")
                    self.goal = None
                    command.angular.x = 0
                    command.angular.z = 0
                    

                self.vel.publish(command)

            self.rate.sleep()

    def main(self):
        self.turtle_control_loop()

if __name__ == "__main__":
    try:
        move_turtle = TurtleMovementController()
        move_turtle.main()
    except rospy.ROSInterruptException:
        pass