import gym
import gym_argos

env = gym.make('ARGoS-v0')

env.reset()
env.render()