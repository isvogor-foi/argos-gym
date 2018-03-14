import gym
import gym_argos

env = gym.make('ARGoS-v0')
env.reset()

for i in range(100000):
    env.render()
    #env.step()




