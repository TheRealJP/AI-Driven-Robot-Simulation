#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def input(data):
    # rospy.loginfo(data.axes)
    twist = Twist()

    # log joystick input
    rospy.loginfo(str.format("0: {}", data.axes[0]))  # draaien
    rospy.loginfo(str.format("1: {}", data.axes[1]))  # rechtdoor
    # print("0:" + data.axes[0]) <---- werkt niet!
    # print("1:" + data.axes[1]) <---- werkt niet! gebruik loginfo

    # only move if these conditions are met
    if data.axes[1] > 0.5:
        twist.linear.x = 4 * data.axes[1]

    if data.axes[0] > 0.5 or data.axes[0] < -0.5:
        twist.angular.z = 4 * data.axes[0]

    pub.publish(twist)


# Intializes everything
def start():

    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, input)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()


if __name__ == '__main__':
    start()
