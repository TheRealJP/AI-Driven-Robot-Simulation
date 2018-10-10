#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi


class TurnTurtle():
    def __init__(self):
        # Give the node a name
        rospy.init_node('TurnTurtle', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)

        # Publisher to control Turtle's speed
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

        # How fast will we update the movement?
        rate = 10
        r = rospy.Rate(rate)

        # Set angular speed
        angular_speed = 0.3

        # Duration and ticks corresponding to 90 degrees
        angular_duration = (pi / 2.0) / angular_speed
        ticks = int(angular_duration * rate)

        # Initialize the movement command and set rotation speed
        move_cmd = Twist()
        move_cmd.angular.z = angular_speed

        # Now rotate left 90 degrees
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            r.sleep()

    def shutdown(self):
        rospy.loginfo("Stopping Turtle")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        TurnTurtle()
    except:
        rospy.loginfo("MoveSquare node terminated.")
