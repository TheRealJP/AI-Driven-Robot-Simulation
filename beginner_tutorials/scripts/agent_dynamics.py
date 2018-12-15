#! /usr/bin/env python

# ros gaat niet verder dan scripts zoeken
# from agent_environment import AgentEnvironment

import rospy
from math import pi
from math import isnan
from math import sqrt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import csv
import math

from agent_environment import AgentEnvironment

"""
todo: translate states to coordinates on the go? starting with the current coordinate as a parameter
for example: starting on [0.5,0.5]  or  self.optimal_path[0].state next state in that direction [0.5 ,0.5+1] 
"""


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


# subscribers in commentaar --> geen actie meer, blijft wachten
# dus init node name niet echt invloed
# cmd_vel gebruiken..
class Robot:
    def __init__(self, topic, threshold, linear_speed, angular_speed, rate, env):
        # Init
        rospy.init_node('run_ai_robot', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # Publishers (ouput)
        self.__cmd_vel = rospy.Publisher(topic, Twist, queue_size=1)

        # Parameters
        self.__threshold = threshold
        self.__linear_speed = linear_speed
        self.__angular_speed = angular_speed
        self.__move_cmd = Twist()
        self.__rate = rate
        self.__ticks = 0
        self.__current_tick = 0
        self.__turning = False
        rospy.Rate(rate)

        # Direction & Rotationdatax
        self.robot_env = env
        self.robot_env.fill_optimal_path()
        self.action = int(self.robot_env.direction_facing)  # first action

        # Position robot
        # scan
        self.__scanner = rospy.Subscriber('/scan', LaserScan, self.set_cmd_vel)  # i'm subscribing on this topic
        rospy.loginfo('wait')
        rospy.wait_for_message('/scan', LaserScan)

        # odom
        rospy.Subscriber('/odom', Odometry, self.get_odom)

        # Spin
        rospy.loginfo('spin')
        rospy.spin()

    # callback odom
    def get_odom(self, odom_data):
        # Callback function for /odom topic
        return odom_data.pose.pose.position

    # callback scan
    def set_cmd_vel(self, msg):
        rospy.loginfo('Turning: %s; Ticks: %s / %s', str(self.__turning), str(self.__current_tick), str(self.__ticks))

        # scan distance to wall from the camerapoint
        can_move, dist = self.scan(msg)
        next_position = 0
        # scanner gives back total distance from where you stand
        #
        # scan_dist - dist_todo = going_to_this_dist --> do this one time when you stand still (linear speed is 0...)
        # amount_todo_now >= scan_dist

        # Move forward if possible and when not turning
        if can_move and not self.__turning:

            # if the speed is 0 measure the next point to reach
            if self.__move_cmd.linear.x is 0:
                next_position = dist - 1

            # check if we reached goal
            if next_position > dist:

                # update to the next action
                self.action = self.robot_env.step(self.action)

            # todo: odometry one meter forward
            rospy.loginfo('move forward')
            self.__move_cmd.angular.z = 0
            self.__move_cmd.linear.x = self.__linear_speed
            self.__cmd_vel.publish(self.__move_cmd)


        # Else turn
        else:
            rospy.loginfo('turn')
            self.__move_cmd.linear.x = 0
            self.__move_cmd.angular.z = self.__angular_speed
            self.turn()

    def scan_distance(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        return dist

    def scan(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        return dist > self.__threshold, dist

    #    signal that you have arrived (something like stopped its ticks)
    def turn(self):
        if self.__current_tick < 1:
            # returns radians to be turned with a given action
            angle = self.robot_env.rotate(self.action)
            rospy.loginfo(angle)
            rospy.loginfo('turning %s radians (90 degrees)', angle)
            angular_duration = angle / self.__angular_speed
            self.__ticks = int(angular_duration * self.__rate)
            self.__turning = True
            self.__current_tick = 1
        elif self.__current_tick >= self.__ticks:
            self.__current_tick = 0
            self.__turning = False
        else:
            angle = pi / 2
            rospy.loginfo('turning %s radians (90 degrees)', angle)
            rospy.loginfo('turning at %s radians / s', str(self.__move_cmd.angular.z))
            self.__cmd_vel.publish(self.__move_cmd)
            self.__current_tick += 1

    def shutdown(self):
        rospy.loginfo('Stopping Roomba')
        self.__cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        env = AgentEnvironment(4, 4, 15)
        roomba = Robot('/mobile_base/commands/velocity', 1, .2, .3, 10, env)
    except:
        rospy.loginfo('Roomba node terminated.')
