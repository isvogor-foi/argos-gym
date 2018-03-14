import gym
from gym import spaces
from argos import argos_runner, argos_io


class ArgosEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.argos = argos_runner.Argos()
        self.argos_io = argos_io.ArgosIO(num_robots=3, verbose=True)
        pass

    def render(self, mode='human'):
        self.argos_io.send_to("yo...", 0)
        self.argos_io.receive_from(0)
        print("Rendering...")


    def step(self, action):
        pass

    def reset(self):
        pass

    def _take_action(self, action):
        pass

    def _get_reward(self):
        """ Reward is given for XY. """
        if self.status == FOOBAR:
            return 1
        elif self.status == ABC:
            return self.somestate ** 2
        else:
            return 0