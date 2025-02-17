#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from turtlesim.msg import Pose

class UVLightController:
    def __init__(self):
        rospy.init_node('uv_light_controller', anonymous=True)

        # Publisher for the UV light status
        self.publisher = rospy.Publisher('/disinfecting', Bool, queue_size=10)

        # Subscriber to the turtle's pose
        rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)

        # Rate for publishing at 10Hz
        self.rate = rospy.Rate(10)

        # Variable to track the UV light status
        self.light_on = False

    def pose_callback(self, pose: Pose):
        # Check if the turtle is near the (1,1) position
        if abs(pose.x - 1.0) < 0.1 and abs(pose.y - 1.0) < 0.1:
            self.light_on = True  # Turn on the light
        else:
            self.light_on = False  # Turn off the light

    def run(self):
        rospy.loginfo("UV Light Controller running...")
        while not rospy.is_shutdown():
            # Publish the UV light status at 10Hz
            self.publisher.publish(Bool(data=self.light_on))
            self.rate.sleep()

if __name__ == "__main__":
    try:
        controller = UVLightController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("UV Light Controller shut down.")

