import gym
import numpy as np
from gym import spaces
from argos import argos_runner, argos_io


class ArgosEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # left-right speed
        #self.action_space = spaces.Box(np.array([0, 0]), np.array([+50, +50]))
        self.action_space = spaces.Box(np.array([-10, -10]), np.array([+10, +10]))
        self.state = None
        self.l_v = 1
        self.r_v = 1
        self.done = False

    def set_argos(self, argos_io, robot_id):
        self.robot_id = robot_id
        self.argos_io = argos_io

    def render(self, mode='human'):
        pass

    def step(self, action):
        # in messages
        if self.done:
            msg = str(0.0) + ";" + str(0.0)
            self.argos_io.send_to(msg, self.robot_id)
            return 0, 0, self.done, {}

        in_msg = self.argos_io.receive_from(self.robot_id)
        in_msg = in_msg.split(";")
        # get floor color
        floor = float(in_msg[2].replace('\x00', ''))
        old_state = self.state
        self.state = floor
        if old_state is not None and old_state > floor:
            self.l_v = 1
            self.r_v = 1
            reward = 10
        else:
            self.l_v = round(action[0], 4)
            self.r_v = round(action[1], 4)
            reward = 0

        if floor <= 25:
            self.done = True

        # out messages
        msg = str(self.l_v) + ";" + str(self.r_v)
        self.argos_io.send_to(msg, self.robot_id)

        return floor, reward, self.done, {}

    def reset(self):
        pass

    def _take_action(self, action):
        pass

    def _get_reward(self):
        pass