import csv
import math

from Skynet_alpha.ai.policy import Policy


class AgentEnvironment:

    def __init__(self, row, column, treasure_state):

        self.row = row
        self.column = column
        self.treasure_state = treasure_state
        self.current_state = 0
        self.optimal_path = []
        self.direction_facing = 2  # starting direction of robot
        self.pos_rotation = False

    def step(self, action):
        """
        robot puts a step in the given action/direction
        change state and face_direction
        :param action: current action for the robot to take
        :return: next_action
        """
        # todo: replace with opencv bool:
        if self.current_state <= (len(self.optimal_path) - 1) \
                or self.current_state is self.treasure_state:
            # get the next state & action by moving the robot
            next_action, next_state = self.move_robot(action)
            self.current_state = next_state  # update state

            return int(next_action)

    def move_robot(self, direction):
        """
        move in the optimal path array
        :param direction:
        :return: next_action,next_state
        """
        if self.current_state >= len(self.optimal_path) - 1:
            return 0, 0

        # "reposition" index/status of robot
        if direction == 0:  # left
            index = self.current_state - 1
        elif direction == 1:  # down
            index = self.current_state + self.row  # length of row
        elif direction == 2:  # right
            index = self.current_state + 1
        elif direction == 3:  # up
            index = self.current_state - self.row
        else:
            index = 0

        # set the action as the current direction of the front of the robot
        # self.direction_facing = direction

        # get the action and state at the next position
        # if its smaller
        next_action = self.optimal_path[index if index >= 0 else self.current_state].action
        next_state = self.optimal_path[index if index >= 0 else self.current_state].state
        return int(next_action), int(next_state)

    def rotate(self, new_direction):
        """
        returns amount of radians to turn
        :param new_direction: the next direction the robot will face
        :return: degrees in radians
        """

        # assure that there still are next states
        if self.current_state >= len(self.optimal_path) - 1:
            return 0

        if new_direction is 0:  # LEFT
            return math.pi
        if new_direction is 1:  # DOWN
            return -1.57
        if new_direction is 2:  # RIGHT
            return -0
        if new_direction is 3:  # UP
            return 1.57

    def fill_optimal_path(self):
        """
        extract the optimal path
        :return:
        """
        with open(
                '/home/jonathanpeers/catkin_ws/src/ROS_Robotics/beginner_tutorials/scripts/Skynet_alpha'
                '/voorbeeld_policy.csv',
                'r') as f:
            reader = csv.reader(f)
            for file_row in reader:
                if float(file_row[2]) > 0.5:
                    p = Policy(file_row[0], file_row[1], file_row[2])
                    self.optimal_path.append(p)
