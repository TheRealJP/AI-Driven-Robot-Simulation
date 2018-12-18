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

from tf.transformations import euler_from_quaternion

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
# custom world file launch command:
# roslaunch turtlebot_gazebo turtlebot_world.launch world_
# file:=/home/gandalf/catkin_ws/src/ROS_Robotics/beginner_tutorials/worlds/static_boxes


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

        rospy.Rate(rate)

        # Direction & Rotationdatax
        self.robot_env = env
        self.robot_env.fill_optimal_path()
        self.action = self.get_action_current_state()

        # move
        self.next_position = 0
        self.__can_move = True
        self.__dist = 0
        self.__pos = None
        self.__y_start = 0
        self.__x_start = 0
        self.__goal_distance = 1
        self.__scan_dist = 0

        # rotate
        self.__angle = 0
        self.__ticks = 0
        self.__current_tick = 0
        self.__turning = False
        self.__roll = self.__pitch = self.__yaw = 0.0
        self.__turn_precision = 0.01
        self.__error_factor = 0.5

        # Position robot
        # scan, for not bumping into wall
        self.scanner = rospy.Subscriber('/scan', LaserScan, self.callback_scan)  # i'm subscribing on this topic
        rospy.loginfo('wait')
        rospy.wait_for_message('/scan', LaserScan)

        # odom, for tracking distance done
        self.__odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        # Spin
        rospy.loginfo('spin')
        rospy.spin()

    def calc_euclidian_distance(self):
        # first round it doesnt get a distance
        return sqrt(pow((self.__pos.x - self.__x_start), 2) +
                    pow((self.__pos.y - self.__y_start), 2))

    def get_action_current_state(self):
        current_state = self.robot_env.current_state
        return int(self.robot_env.optimal_path[current_state].action)

    def callback_odom(self, msg):
        # rospy.loginfo('Turning: %s; Ticks: %s / %s', str(self.__turning), str(self.__current_tick), str(self.__ticks))

        self.__pos = msg.pose.pose.position

        """
        $scanner gives back total distance from where you stand
        scan_dist - dist_todo = going_to_this_dist --> do this one time when you stand still (linear speed is 0...)
        amount_todo_now >= scan_dist
        
        Move forward if possible and when not turning and when you passed the set distance goal
        """
        has_moving_ended = self.__goal_distance > self.__dist
        turn_first = self.action != self.get_action_current_state()
        if self.__can_move and not self.__turning and has_moving_ended and not turn_first:
            self.move()
        else:
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.__roll, self.__pitch, self.__yaw = euler_from_quaternion(orientation_list)
            self.turn()

    def move(self):
        # setting up everything before starting (think calc_euclidian_distance())
        if self.__move_cmd.linear.x == 0.0:
            # step gives back the next action based on the current action
            self.action = self.robot_env.step(self.get_action_current_state())

            # save start of the movement
            self.__x_start = self.__pos.x
            self.__y_start = self.__pos.y
            self.__move_cmd.angular.z = 0
            self.__move_cmd.linear.x = self.__linear_speed
            return
        else:
            rospy.loginfo('move forward--> current location; x -> %s , y -> %s', self.__pos.x, self.__pos.y)
            rospy.loginfo('distance done by robot: %s', self.__dist)

            self.__cmd_vel.publish(self.__move_cmd)
            speed = self.__goal_distance - self.__dist

            # lowest speed limit or decrementing speed value
            self.__move_cmd.linear.x = speed if speed > self.__linear_speed else self.__linear_speed

            # robot moved so now we calculate the distance
            self.__dist = self.calc_euclidian_distance()

    def turn(self):
        difference = abs(self.__angle - self.__yaw)
        rospy.loginfo('yaw: %s | difference: %s', self.__yaw, difference)

        # config for the start of the turn
        if self.__current_tick < 1:
            # fetch angle (doesnt stay filled for some reason when you isolate it inside an if statement)

            # todo translate angles to gazebo
            # -1.57 = 0
            # 1.57 = math.pi
            #  0 = 1.57
            # -1.57 = 3.14

            # angle     |  gazebo
            # math.pi   |  3.14
            # 0         |  0
            # -1.57     |  1.57
            # 1.57      |  -1.57

            self.__angle = 1.57
            # self.robot_env.rotate(self.action)
            self.__move_cmd.linear.x = 0
            # self.__move_cmd.angular.z = self.__angular_speed if self.robot_env.pos_rotation else -self.__angular_speed

            # returns radians to be turned with a given action
            rospy.loginfo('turning %s radians (%s degrees)', self.__angle, self.__angle * 180 / math.pi)

            # how long will it take to turn
            # angular_duration = self.__angle / self.__angular_speed
            # self.__ticks = abs(int(angular_duration * self.__rate))

            self.__turning = True
            self.__current_tick = 1

            # bug: -52 ticks < 1 tick

        # stop turning
        # elif self.__current_tick >= self.__ticks:
        elif difference <= self.__turn_precision:
            self.__current_tick = 0
            self.__move_cmd.angular.z = 0
            self.__turning = False
            # self.__cmd_vel.publish(self.__move_cmd)
            rospy.loginfo("Finished turning!")

        # during the turn
        else:
            rospy.loginfo('continueing to %s radians (%s degrees)', self.__angle, self.__angle * 180 / math.pi)
            # self.__linear_speed = 0.1 if 0.1 > abs(difference) else abs(difference)
            # self.__move_cmd.angular.z = self.__angular_speed if self.robot_env.pos_rotation else -self.__angular_speed
            self.__move_cmd.angular.z = difference * 0.45
            # self.__angular_speed if self.robot_env.pos_rotation else -self.__angular_speed
            rospy.loginfo('turning at %s radians / s', str(self.__move_cmd.angular.z))

            self.__cmd_vel.publish(self.__move_cmd)
            self.__current_tick += 1

    def shutdown(self):
        rospy.loginfo('Stopping Roomba')
        self.__cmd_vel.publish(Twist())
        rospy.sleep(1)

    def callback_scan(self, msg):
        if msg.header.frame_id == "/camera_depth_frame":
            self.__can_move, self.__scan_dist = self.scan(msg)
            # rospy.loginfo('multiple point: %s', avg_minimum(msg.ranges[315:330], len(msg.ranges[315:330])))

    def scan(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        # rospy.loginfo('msg distance: %s', avg_minimum(msg.ranges, len(msg.ranges) / 10))
        return dist > self.__threshold, dist


if __name__ == '__main__':
    try:
        env = AgentEnvironment(4, 4, 15)
        # x = -1,37 with linear_speed= 0.3
        # x = -1,43 with linear_speed= 0.2
        roomba = Robot('/mobile_base/commands/velocity', 1, .1, .1, 10, env)
    except:
        rospy.loginfo('Roomba node terminated.')
