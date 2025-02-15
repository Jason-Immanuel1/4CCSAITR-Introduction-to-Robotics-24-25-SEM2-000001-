#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def publish_goal():
    # Initialize the ROS node
    rospy.init_node('goal_publisher', anonymous=True)

    # Create a publisher for the /goal topic
    goal_pub = rospy.Publisher('/goal', Point, queue_size=10)

    # Set the publishing rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        try:

            # Create a Point message
            goal = Point(1, 1, 0.0)

            # Publish the goal
            goal_pub.publish(goal)


        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values.")

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass