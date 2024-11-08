#!/usr/bin/env python

import rospy
import actionlib
import sys
from multimap_navigation.msg import MultimapNavigationActionAction, MultimapNavigationActionGoal

def send_goal(target_map, target_x, target_y, target_yaw):
    # Initialize the ROS node
    rospy.init_node('multimap_navigation_client')

    # Create the SimpleActionClient
    client = actionlib.SimpleActionClient('multimap_navigation', MultimapNavigationActionAction)

    rospy.loginfo("Waiting for the action server to start...")
    client.wait_for_server()

    # Create and populate the goal
    goal = MultimapNavigationActionGoal()
    goal.target_map = target_map
    # goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_x = target_x
    goal.target_y = target_y
    goal.target_yaw = target_yaw

    rospy.loginfo(f"Sending goal: map={target_map}, x={target_x}, y={target_y}, orientation_w={target_yaw}")
    client.send_goal(goal)

    # Wait for result
    client.wait_for_result(rospy.Duration(30.0))

    # Check result
    if client.get_result():
        rospy.loginfo("Goal reached successfully.")
    else:
        rospy.logwarn("Action did not complete successfully or within time limit.")

if __name__ == '__main__':
    # Command-line arguments: map_name x y orientation_w
    if len(sys.argv) != 5:
        print("Usage: rosrun multimap_navigation multimap_navigation_client.py <map_name> <x> <y> <orientation_w>")
        sys.exit(1)

    map_name = sys.argv[1]
    try:
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        orientation_w = float(sys.argv[4])
    except ValueError:
        rospy.logerr("Coordinates and orientation must be valid numbers.")
        sys.exit(1)

    try:
        send_goal(map_name, x, y, orientation_w)
    except rospy.ROSInterruptException:
        rospy.logerr("Navigation interrupted.")

