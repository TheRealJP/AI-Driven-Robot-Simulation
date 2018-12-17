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
    # rospy.loginfo('dist: %s, nbr of points %s', str(dist), str(n))

    return dist


# subscribers in commentaar --> geen actie meer, blijft wachten
# dus init node name niet echt invloed
# cmd_vel gebruiken..
# does scan and odom work together nicely?
# odom for precise tracking
# scanner for not bumping into wall
# -------
# roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/gandalf/catkin_ws/src/ROS_Robotics/beginner_tutorials/worlds/static_boxes


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
        self.action = self.robot_env.direction_facing
        self.angle = 0
        self.next_position = 0

        # Position robot
        # scan, for not bumping into wall
        self.scanner = rospy.Subscriber('/kobuki/laser/scan', LaserScan,
                                        self.set_cmd_vel)  # i'm subscribing on this topic
        rospy.loginfo('wait')
        rospy.wait_for_message('/kobuki/laser/scan', LaserScan)

        # Spin
        rospy.loginfo('spin')
        rospy.spin()

    # callback odom
    def get_odom(self, odom_data):
        # Callback function for /odom topic
        return odom_data.pose.pose.position

    # callback scan
    def set_cmd_vel(self, msg):
        # rospy.loginfo(len(msg.ranges))
        rospy.loginfo(msg.ranges[320])
        # rospy.loginfo('one point: %s', msg.ranges[320])
        # rospy.loginfo('multiple point: %s', avg_minimum(msg.ranges[315:330], len(msg.ranges[315:330])))  # straight in middle
        # rospy.loginfo('Turning: %s; Ticks: %s / %s', str(self.__turning), str(self.__current_tick), str(self.__ticks))

        # "dist" is scan_distance to the wall based from the camerapoint
        can_move, dist = self.scan(msg)
        # scanner gives back total distance from where you stand
        # scan_dist - dist_todo = going_to_this_dist --> do this one time when you stand still (linear speed is 0...)
        # amount_todo_now >= scan_dist

        # Move forward if possible and when not turning and when you passed the set distance goal
        if can_move and not self.__turning and self.next_position < dist:

            # if the speed is 0 measure the next point to reach
            if self.__move_cmd.linear.x == 0.0:
                # update next distance
                self.next_position = dist - 1
                rospy.loginfo('next_position: %s', self.next_position)

            # step gives back the next action based on the current action
            current_state = self.robot_env.current_state
            current_action = self.robot_env.optimal_path[current_state].action
            self.action = self.robot_env.step(current_action)

            # rospy.loginfo('move forward')
            self.__move_cmd.angular.z = 0
            self.__move_cmd.linear.x = self.__linear_speed
            self.__cmd_vel.publish(self.__move_cmd)
        # Else turn
        else:
            # todo check if you still need to turn based on current radians

            rospy.loginfo('turn')
            self.__move_cmd.linear.x = 0
            self.__move_cmd.angular.z = self.__angular_speed
            self.turn()

        msg.ranges = []

    def scan_distance(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        return dist

    def scan(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        # rospy.loginfo('msg distance: %s', avg_minimum(msg.ranges, len(msg.ranges) / 10))
        return dist > self.__threshold, dist

    #    signal that you have arrived (something like stopped its ticks)
    def turn(self):

        if self.__current_tick < 1:
            # returns radians to be turned with a given action
            self.angle = self.robot_env.rotate(self.action)

            rospy.loginfo('turning %s radians (%s degrees)',
                          self.angle, self.angle * 180 / math.pi)
            angular_duration = self.angle / self.__angular_speed
            self.__ticks = int(angular_duration * self.__rate)
            self.__turning = True
            self.__current_tick = 1

        elif self.__current_tick >= self.__ticks:
            self.__current_tick = 0
            self.__turning = False
        else:
            rospy.loginfo('turning %s radians (%s degrees)',
                          self.angle, self.angle * 180 / math.pi)
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
        roomba.scanner = rospy.Subscriber('/kobuki/laser/scan', LaserScan,
                                          roomba.set_cmd_vel)  # i'm subscribing on this topic

    except:
        rospy.loginfo('Roomba node terminated.')
