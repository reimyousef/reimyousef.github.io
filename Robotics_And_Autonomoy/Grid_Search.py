#!/usr/bin/env python3
"""
grid_search_node.py
----------------------------------------
ROS-based Grid Search Navigation Node

This script enables rover to perform a simple grid search pattern using odometry feedback and waypoint navigation. 
The node publishes movement commands to guide the robot through predefined waypoints while continuously checking for an ArUco marker detection signal.

Features:
- Subscribes to odometry and ArUco marker detection topics
- Computes angle and distance to goal using current pose
- Executes heading adjustment + forward drive behavior
- Stops immediately when a marker is detected or all waypoints are visited
- Publishes velocity commands via /drive topic


Platform: ROS (Python 3)
"""

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool, Float32

# Global robot state
x = 0.0
y = 0.0
theta = 0.0
found = False
scale_factor = 1.0

def new_odom(msg):
    """Callback for Odometry messages. Updates global robot position and orientation."""
    global x, y, theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def aruco_callback(msg):
    """Callback for ArUco detection."""
    global found
    if msg.data:
        rospy.loginfo("ArUco marker found.")
        found = True

def dist_to_goal(goal_x, goal_y):
    """Returns distance from current position to the goal."""
    dx = (goal_x - x) * scale_factor
    dy = (goal_y - y) * scale_factor
    return np.sqrt(dx**2 + dy**2)

def ang_to_goal(goal_x, goal_y):
    """Returns bearing from current orientation to the goal."""
    dx = (goal_x - x) * scale_factor
    dy = (goal_y - y) * scale_factor
    return np.arctan2(dy, dx)

def stop_robot(pub, speed_msg):
    """Sends zero velocity to stop the robot."""
    speed_msg.linear.x = 0.0
    speed_msg.angular.z = 0.0
    pub.publish(speed_msg)

def main():
    global found

    rospy.init_node("grid_search")

    rospy.Subscriber("/rtabmap/odom", Odometry, new_odom)
    rospy.Subscriber("aruco_found", Bool, aruco_callback)

    cmd_pub = rospy.Publisher("drive", Twist, queue_size=1)
    err_pub = rospy.Publisher("/robot_base_velocity_controller/error", Float32, queue_size=1)

    speed = Twist()
    rate = rospy.Rate(10)

    # define grid points (relative to initial pose)
    path = [
        (3.5, 0.0),
        (3.5, 3.5),
        (-3.5, 3.5),
        # add more points here as needed
    ]

    rospy.loginfo("Starting grid search...")

    while path and not found and not rospy.is_shutdown():
        goal_x, goal_y = path.pop(0)

        distance = dist_to_goal(goal_x, goal_y)
        angle = ang_to_goal(goal_x, goal_y)

        rospy.loginfo(f"Navigating to ({goal_x:.2f}, {goal_y:.2f})")

        while distance > 0.5 and not found and not rospy.is_shutdown():
            diff_angle = angle - theta

            if abs(diff_angle) > 0.15:
                speed.linear.x = 0.0
                speed.angular.z = 0.3 if diff_angle > 0 else -0.3
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

            cmd_pub.publish(speed)
            distance = dist_to_goal(goal_x, goal_y)
            angle = ang_to_goal(goal_x, goal_y)
            rate.sleep()

    stop_robot(cmd_pub, speed)

    if found:
        rospy.loginfo("Grid search complete: ArUco marker found.")
    else:
        rospy.loginfo("Grid search complete: No marker detected.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass