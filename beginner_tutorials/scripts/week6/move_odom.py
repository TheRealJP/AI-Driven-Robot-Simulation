#!/usr/bin/env python

""" move_odom.py    - G. De Paepe
                    - Version 0.1
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow


class MoveOdom():
    def __init__(self):
        # Give the node a name
        rospy.init_node('move_odom', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

        # How fast will we update the robot's movement?
        rate = 50
        r = rospy.Rate(rate)

        # Set the linear and the goal distance
        linear_speed = 0.2
        goal_distance = 1.0

        # Subscribe to /odom topic and set callback function
        rospy.Subscriber('/odom', Odometry, self.get_odom)

        # Initialize the movement command
        move_cmd = Twist()
        rospy.sleep(1)

        # Track current position
        x_start = self.position.x
        y_start = self.position.y

        # Move forward goal_distance meters
        distance = 0
        move_cmd.linear.x = linear_speed
        while distance < goal_distance:
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((self.position.x - x_start), 2) +
                            pow((self.position.y - y_start), 2))

        # Stop the robot
        self.cmd_vel.publish(Twist())

    def get_odom(self, odom_data):
        # Callback function for /odom topic
        self.position = odom_data.pose.pose.position

    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        MoveOdom()
    except:
        rospy.loginfo("move_odom node terminated.")
