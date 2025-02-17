#!/usr/bin/env python3

import rospy
from path_planner import PathPlanner
from turtle_movement_controller import TurtleMovementController
from turtle_goal_publisher import TurtleGoalPublisher
from uv_light_controller import UVLightController


class MainNode:
    def __init__(self):
        rospy.init_node('main_node', anonymous=True)

        # Instantiate the classes
        self.planner = PathPlanner()  # Create an instance of PathPlanner
        self.movement = TurtleMovementController()  # Create an instance of TurtleMovementController
        self.publisher = TurtleGoalPublisher()  # Create an instance of TurtleGoalPublisher
        self.uv_light = UVLightController()  # Create an instance of UVLightController

    def main(self):
        # Call the main method of PathPlanner and get the path
        path = self.planner.main()
        if path:
            print("Path received from PathPlanner:", path)
        else:
            print("No path returned from PathPlanner.")


if __name__ == '__main__':
    try:
        main_node = MainNode()
        main_node.main()
    except rospy.ROSInterruptException:
        pass
        
