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
        self.action = self.robot_env.direction_facing

        # move
        self.next_position = 0
        self.__can_move = True
        self.__dist = 0
        self.__position = None
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
        self.__turn_precision = 0.045

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
        return sqrt(pow((self.__position.x - self.__x_start), 2) +
                    pow((self.__position.y - self.__y_start), 2))

    def callback_scan(self, msg):
        if msg.header.frame_id == "/camera_depth_frame":
            # "dist" is scan_distance to the wall based from the camerapoint
            self.__can_move, self.__scan_dist = self.scan(msg)
            rospy.loginfo('multiple point: %s', avg_minimum(msg.ranges[315:330], len(msg.ranges[315:330])))
            # straight in middle

    # callback scan
    def callback_odom(self, msg):
        # rospy.loginfo('Turning: %s; Ticks: %s / %s', str(self.__turning), str(self.__current_tick), str(self.__ticks))

        if msg.header.frame_id == "odom":
            self.__position = msg.pose.pose.position
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.__roll, self.__pitch, self.__yaw = euler_from_quaternion(orientation_list)

        """
        $scanner gives back total distance from where you stand
        scan_dist - dist_todo = going_to_this_dist --> do this one time when you stand still (linear speed is 0...)
        amount_todo_now >= scan_dist
        
        Move forward if possible and when not turning and when you passed the set distance goal
        """
        has_moving_ended = self.__goal_distance > self.__dist

        if self.__can_move and not self.__turning and has_moving_ended:
            self.move()
        else:
            self.turn()

    def scan(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        # rospy.loginfo('msg distance: %s', avg_minimum(msg.ranges, len(msg.ranges) / 10))
        return dist > self.__threshold, dist

    def turn(self):
        rospy.loginfo('turn')
        self.__angle = self.robot_env.rotate(self.action)

        # config for the start of the turn
        if self.__current_tick < 1:
            self.__dist = 0  # reset moving distance of the robot
            self.__move_cmd.linear.x = 0
            self.__move_cmd.angular.z = self.__angular_speed

            # returns radians to be turned with a given action
            rospy.loginfo('turning %s radians (%s degrees)', self.__angle, self.__angle * 180 / math.pi)

            # how long will it take to turn
            angular_duration = self.__angle / self.__angular_speed
            self.__ticks = int(angular_duration * self.__rate)

            self.__turning = True
            self.__current_tick = 1

        # stop turning
        elif self.__current_tick >= self.__ticks:
            self.__current_tick = 0
            self.__turning = False

        # during the turn
        else:
            rospy.loginfo('turning %s radians (%s degrees)', self.__angle, self.__angle * 180 / math.pi)
            rospy.loginfo('turning at %s radians / s', str(self.__move_cmd.angular.z))
            self.__cmd_vel.publish(self.__move_cmd)
            self.__current_tick += 1

    def shutdown(self):
        rospy.loginfo('Stopping Roomba')
        self.__cmd_vel.publish(Twist())
        rospy.sleep(1)

    def move(self):
        if self.__move_cmd.linear.x == 0.0:
            # update next distance
            #     self.next_position = self.__dist - 1
            #     rospy.loginfo('next_position: %s', self.next_position)

            # step gives back the next action based on the current action
            current_state = self.robot_env.current_state
            current_action = self.robot_env.optimal_path[current_state].action
            self.action = self.robot_env.step(current_action)

            # save start of the movement
            self.__x_start = self.__position.x
            self.__y_start = self.__position.y
            self.__move_cmd.angular.z = 0
            self.__move_cmd.linear.x = self.__linear_speed
            return

        if self.__move_cmd.linear.x != 0.0:
            rospy.loginfo('move forward--> current location; x -> %s , y -> %s', self.__position.x,
                          self.__position.y)
            rospy.loginfo('distance done by robot: %s', self.__dist)
            self.__cmd_vel.publish(self.__move_cmd)
            speed = self.__goal_distance - self.__dist
            self.__move_cmd.linear.x = speed if speed > self.__linear_speed else self.__linear_speed  # 1 - 0 = 0

            # if msg.header.frame_id == "odom":
            # robot moved so now we calculate the distance
        self.__dist = self.calc_euclidian_distance()


if __name__ == '__main__':
    try:
        env = AgentEnvironment(4, 4, 15)

        # x = -1,37 with linear_speed= 0.3
        # x = -1,43 with linear_speed= 0.2
        roomba = Robot('/mobile_base/commands/velocity', 1, .1, .3, 10, env)
    except:
        rospy.loginfo('Roomba node terminated.')
