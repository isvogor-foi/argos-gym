import gym
import numpy as np
from gym import spaces
from argos import argos_runner, argos_io


class ArgosEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.action_space = spaces.Box(np.array([-50, -50]), np.array([+50, +50]))

    def set_argos(self, argos, argos_io):
        self.argos = argos #argos_runner.Argos()
        self.argos_io = argos_io #argos_io.ArgosIO(num_robots=3, verbose=True)

    def render(self, mode='human'):
        pass

    def step(self, action):
        # out messages
        msg = str(action[0]) + ";" + str(action[1])
        self.argos_io.send_to(msg, 0)
        #print("action: ", action)

        #in messages
        in_msg = self.argos_io.receive_from(0)
        in_msg = in_msg.split(";")
        floor = float(in_msg[2].replace('\x00', ''))
        #print("Color: ", str(floor))
        if action[0] > 0 and action [1] > 0:
            reward = floor + 500
        else:
            reward = floor


        return floor, reward, False, {}

    def reset(self):
        pass

    def _take_action(self, action):
        pass

    def _get_reward(self):
        pass