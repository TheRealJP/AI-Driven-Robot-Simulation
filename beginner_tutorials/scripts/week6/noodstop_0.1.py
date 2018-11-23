#!/usr/bin/env python
#
#    noodstop_0.1.py - Version 0.1 G. De Paepe
#        
#    robot beweegt in x-richting met snelheid "speed"
#    robot stop:
#       - als hij minder dan een "threshold" afstand van een obstakel komt
#       - wanneer er geen scan informatie is
#    afstandsberekening:
#       - neemt gemiddelde van "nbr_mid" middelste punten in laserscan
#
import random
import rospy
from roslib import message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import isnan, pi


class NoodStop:
    def __init__(self):
        rospy.init_node("noodstop")
        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        # How far away (in m) before the robot stops
        self.threshold = 0.8
        # The speed in meters per second
        self.speed = 0.1
        # Set rate to update robot's movement
        self.rate = 100
        self.r = rospy.Rate(self.rate)
        # Initialize the movement command
        self.move_cmd = Twist()
        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

        # Subscribe to the laserscan
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.set_cmd_vel)

        rospy.loginfo("Subscribing to laserscan...")

        # Wait for the laserscan topic to become available
        rospy.wait_for_message('/scan', LaserScan)

        rospy.loginfo("Ready to start!")

        while not rospy.is_shutdown():
            # Publish the movement command
            self.cmd_vel_pub.publish(self.move_cmd)
            self.r.sleep()

    def forward(self):
        # How fast will we update the movement?

        # Initialize the movement command and set rotation speed
        self.move_cmd.linear.x = self.speed
        self.move_cmd.angular.z = 0
        rospy.loginfo("moving forward...")

    def rotate(self, speed):
        # How fast will we update the movement?
        rate = 100

        # Initialize the movement command and set rotation speed
        self.move_cmd.angular.z = speed
        self.move_cmd.linear.x = 0
        # Now rotate left 90 degrees
        # for t in range(ticks):
        rospy.loginfo("rotating...")
        # self.cmd_vel_pub.publish(self.move_cmd)

    def set_cmd_vel(self, msg):
        # Initialization
        dist = n = 0

        # Get indexes to filter the "nbr_mid" middle point out of the laserscan points
        len_scan = len(msg.ranges)
        nbr_mid = 50
        if len_scan > nbr_mid:
            midrange_min = int((len_scan - nbr_mid) / 2)
            midrange_max = midrange_min + nbr_mid
        else:
            range_min = 0
            range_max = len_scan = len(msg.ranges)

        # Compute average distance out of the middle points, skip NaN
        for point in msg.ranges[midrange_min:midrange_max]:
            if not isnan(point):
                dist += point
                n += 1
        # If no points, keep dist equal to zero
        if n:
            dist /= n
        rospy.loginfo("dist: %s, nbr of points %s", String(dist), String(n))

        # Move if dist is greater then threshold, else set speed to zero
        if dist > self.threshold:
            self.forward()
        else:
            # Set angular speed (random angle)
            angular_speed = random.uniform(0.1, 0.5)

            # Duration and ticks corresponding to 90 degrees
            angular_duration = (pi / 2.0) / angular_speed
            ticks = int(angular_duration * self.rate)
            for _ in range(ticks):
                self.rotate(angular_speed)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")

        # Unregister the subscriber to stop cmd_vel publishing
        self.scan_subscriber.unregister()
        rospy.sleep(1)

        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        NoodStop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Noodstop node terminated.")
