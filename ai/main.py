import gym
import gym_argos
from argos import argos_runner

env = gym.make('ARGoS-v0')

env.reset()
env.render()

argos = argos_runner.Argos()


#argos.start()