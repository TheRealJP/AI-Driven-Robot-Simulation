#!/usr/bin/env python

import rospy
import random
import sys
from math import pi
from math import isnan
from math import sqrt

from roslib import message
from nav_msgs.msg import Odometry as odom
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from robotics.environment.agent_environment import AgentEnvironment


def dev(x, l):
    n = len(l)
    avg = sum(l) / n
    return (x - avg) ** 2


def std_dev(l):
    n = len(l)
    stddev = 0
    for x in l:
        stddev += dev(x, l)
    return sqrt(stddev / (n - 1))


def avg_minimum(l, n_min):
    dist = n = 0
    stddev = std_dev(l)
    min_dists = [max(l) for _ in range(n_min)]
    max_min_dists = max(min_dists)

    # Compute weighted avg minimum distance, skip NaN
    for point in l:
        if not isnan(point):
            # Get deviation to remove outliers
            d = dev(point, l)
            if (d < 3 * stddev or d > -3 * stddev) and point < max_min_dists:
                min_dists[min_dists.index(max_min_dists)] = point
                max_min_dists = max(min_dists)
        n += 1
    dist = sum(min_dists) / n_min
    rospy.loginfo('dist: %s, nbr of points %s', str(dist), str(n))

    return dist


class Robot:
    def __init__(self, topic, threshold, linear_speed, angular_speed, rate):
        # Init
        rospy.init_node('AI_Robot', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.__cmd_vel = rospy.Publisher(topic, Twist, queue_size=1)

        # Parameters
        self.__threshold = threshold
        self.__linear_speed = linear_speed
        # Suggest using a low angular speed for turning with odometry as speed will be
        # designated from code and adjusted. Maybe not necessary
        self.__angular_speed = angular_speed
        self.__move_cmd = Twist()
        self.__rate = rate
        self.__ticks = 0
        self.__current_tick = 0
        self.__turning = False
        self.__roll = self.__pitch = self.__yaw = 0.0
        self.__turn_precision = 0.045
        rospy.Rate(rate)

        # Subscriptions
        self.__scanner = rospy.Subscriber('/scan', LaserScan, self.set_cmd_vel)
        rospy.loginfo('wait')
        rospy.wait_for_message('/scan', LaserScan)
        self.__odom_subscriber = rospy.Subscriber('/odom', Odometry, self.set_cmd_vel)

        # Spin
        rospy.loginfo('spin')
        rospy.spin()

        # Direction & Rotationdata
        self.robot_env = AgentEnvironment(4, 4, 15)
        self.robot_env.fill_optimal_path()
        self.action = int(self.robot_env.direction_facing)  # first action

    def set_cmd_vel(self, msg):
        rospy.loginfo('Turning: %s; Ticks: %s / %s',
                      str(self.__turning), str(self.__current_tick), str(self.__ticks))
        move = self.scan(msg)

        if msg.header.frame_id == "odom":
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.__roll, self.__pitch, self.__yaw = euler_from_quaternion(orientation_list)


        # Move forward if possible
        if move and not self.__turning:

            # todo: odometry one meter forward
            rospy.loginfo('move forward')
            self.__move_cmd.angular.z = 0
            self.__move_cmd.linear.x = self.__linear_speed
            self.__ticks = 5
            self.__cmd_vel.publish(self.__move_cmd)

            # update to the next action
            self.action = self.robot_env.step(self.action)

        # Else turn
        else:
            # Eigenlijk moet het hier stoppen
            rospy.loginfo('turn')
            self.__move_cmd.linear.x = 0
            self.__move_cmd.angular.z = self.__angular_speed
            self.turn()

    #    signal that you have arrived (something like stopped its ticks)
    # now improved with more precise turning using odometry
    def turn(self):

        target_angle = self.robot_env.rotate(self.action)
        rospy.loginfo('turning %s radians (90 degrees)', target_angle)

        difference = target_angle - self.__yaw
        self.__angular_speed = abs(difference)

        if abs(difference) > self.__turn_precision:
            rospy.loginfo("Turning")
            self.__turning = True
            self.__move_cmd.angular.z = self.__angular_speed
            # To make sure the robot won't be moving forward while turning!
            self.__move_cmd.linear.x = 0
            self.__cmd_vel.publish(self.__move_cmd)

        else:
            rospy.loginfo("Finished turning!")
            self.__turning = False
            self.__move_cmd.angular.z = 0
            # self.__move_cmd.linear.x = 0
            self.__cmd_vel.publish(self.__move_cmd)


    def scan(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        return dist > self.__threshold

    def shutdown(self):
        rospy.loginfo('Stopping Roomba')
        self.__cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        roomba = Robot('/mobile_base/commands/velocity', .5, .2, .3, 10)
    except:
        rospy.loginfo('Roomba node terminated.')
