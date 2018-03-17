import gym
import numpy as np
from gym import spaces
from argos import argos_runner, argos_io


class ArgosEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        # left-right speed
        #self.action_space = spaces.Box(np.array([0, 0]), np.array([+50, +50]))
        self.action_space = spaces.Box(np.array([0, 0]), np.array([+50, +50]))

    def set_argos(self, argos_io, robot_id):
        self.robot_id = robot_id
        self.argos_io = argos_io

    def render(self, mode='human'):
        pass

    def step(self, action):
        # in messages
        in_msg = self.argos_io.receive_from(self.robot_id)
        in_msg = in_msg.split(";")
        # get floor color
        floor = float(in_msg[2].replace('\x00', ''))
        reward = floor
        # out messages
        msg = str(action[0]/10.0) + ";" + str(action[1]/10.0)
        self.argos_io.send_to(msg, self.robot_id)

        #print("action: ", action)

        return floor, reward, False, {}

    def reset(self):
        pass

    def _take_action(self, action):
        pass

    def _get_reward(self):
        pass