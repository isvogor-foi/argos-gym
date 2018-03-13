import gym
import gym_argos
from argos import argos_runner

env = gym.make('ARGoS-v0')
#env2 = gym.make('ARGoS-v0')

env.reset()
env.render()

#env2.reset()
#env2.render()


argos = argos_runner.Argos(num_robots=3)
