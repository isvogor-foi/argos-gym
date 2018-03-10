import gym
from gym import spaces


class ArgosEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        pass

    def render(self, mode='human'):
        print("Rendering...")
        pass

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