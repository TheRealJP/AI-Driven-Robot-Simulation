#! /usr/bin/env python

import rospy
from math import isnan
from math import sqrt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion

from agent_environment import AgentEnvironment


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


"""
debugging: node in commentaar, in pycharm debugger starten --> msg data blijft hetzelfde en gazebo yaw komt niet overeen
rostopic message rate vertragen
todo: scan gebruiken om afstand tot muur te bepalen --> ook gebruiken als stop 
    bv: (self.boolean = (scandistance - todo = 3 -1  = 2 < current distance))

"""


class Robot:
    def __init__(self, topic, threshold, linear_speed, angular_speed, rate, env):
        # Init
        rospy.init_node('run_ai_robot', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.__shutdown_signal = False
        # Publishers (ouput)
        self.__cmd_vel = rospy.Publisher(topic, Twist, queue_size=1)

        # Parameters
        self.__threshold = threshold
        self.__linear_speed = linear_speed
        self.__angular_speed = angular_speed
        self.__move_cmd = Twist()
        self.__rate = rate

        rospy.Rate(rate)

        # Direction & Rotationdata
        self.robot_env = env
        self.robot_env.fill_optimal_path()
        self.action = self.robot_env.direction_facing
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
        self.__current_tick = 0
        self.__turning = False
        self.__roll = self.__pitch = self.__yaw = 0.0
        self.__turn_precision = 0.1
        self.__error_factor = 0.5

        # odom, for tracking distance done
        self.__odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callback_odom)

        # scan, for not bumping into wall
        self.scanner = rospy.Subscriber('/scan', LaserScan, self.callback_scan)  # i'm subscribing on this topic

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

        rospy.loginfo_throttle(period=0.5, msg=('current_action:%s', self.action))
        self.__pos = msg.pose.pose.position

        """
        Move forward if possible and when not turning and when you passed the set distance goal
        """
        has_moving_ended = self.__goal_distance < self.__dist
        if has_moving_ended:
            self.__dist = 0

        turn_first = self.action != self.get_action_current_state()
        movable = self.__can_move and not self.__turning and not has_moving_ended and not turn_first

        if movable:
            self.move()
        else:
            orientation_q = msg.pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            self.__roll, self.__pitch, self.__yaw = euler_from_quaternion(orientation_list)
            self.turn()

            # shutdown when last moving action is done
            if self.__shutdown_signal is True:
                self.shutdown()

    def move(self):
        # setting up everything before starting
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
            rospy.loginfo_throttle(period=0.5, msg=(
            'move forward--> current location; x -> %s , y -> %s', self.__pos.x, self.__pos.y))
            rospy.loginfo_throttle(period=0.5, msg=('distance done by robot: %s', self.__dist))

            speed = self.__goal_distance - self.__dist
            self.__move_cmd.linear.x = speed if speed > 0.08 else 0.08
            self.__cmd_vel.publish(self.__move_cmd)

            # robot moved so now we calculate the distance
            self.__dist = self.calc_euclidian_distance()

            if self.robot_env.current_state is self.robot_env.treasure_state:
                self.__shutdown_signal = True

    # angle     |  gazebo
    # ---------------------
    # math.pi   |  0
    # 0         |  3.14
    # 1.57      | -1.57
    # -1.57     |  1.57
    def turn(self):
        """
        yaw - 0 = yaw --> blijft draaien --> yaw wordt hoger en hoger
        :return:
        """
        has_correct_radians = self.__angle - self.__turn_precision <= self.__yaw <= self.__angle + self.__turn_precision
        difference = abs(self.__angle - self.__yaw)  # yaw 1,57 - angle 0 = 1
        # if difference > 3.14:
        #     difference = difference - 3.14
        # if abs(self.__yaw == difference):
        #     difference = 0.1 - self.__yaw
        rospy.loginfo_throttle(period=0.05,
                               msg=('angle: %s - yaw: %s == difference: %s', self.__angle, self.__yaw, difference))

        if self.__current_tick < 1:  # config for the start of the turn
            self.__angle = self.robot_env.rotate(self.get_action_current_state())
            self.__move_cmd.linear.x = 0
            self.__turning = True
            self.__current_tick = 1

        # elif difference <= self.__turn_precision:  # stop turning
        elif has_correct_radians or difference <= self.__turn_precision:  # stop turning
            self.__current_tick = 0
            self.__move_cmd.angular.z = 0
            self.__turning = False
            # self.__dist = 0
            self.action = self.get_action_current_state()
            rospy.loginfo("Finished turning!")
            # rospy.sleep(1)
        else:  # during the turn

            self.__move_cmd.angular.z = difference  # if difference < 0.1 else 0.1
            rospy.loginfo_throttle(period=0.5, msg=('turning at %s radians / s', str(self.__move_cmd.angular.z)))
            # publish the speed
            self.__cmd_vel.publish(self.__move_cmd)

    def shutdown(self):
        rospy.loginfo('Stopping Roomba')
        self.__cmd_vel.publish(Twist())
        rospy.sleep(1)

    def callback_scan(self, msg):
        self.__can_move = self.scan(msg)

    def scan(self, msg):
        dist = avg_minimum(msg.ranges, len(msg.ranges) / 10)
        return dist > self.__threshold


if __name__ == '__main__':
    try:
        env = AgentEnvironment(4, 4, 15)
        roomba = Robot('/mobile_base/commands/velocity',
                       0.75, .15, .2, 10, env)
    except:
        rospy.loginfo('Roomba node terminated.')
