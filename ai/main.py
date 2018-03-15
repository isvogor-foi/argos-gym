import gym
import gym_argos
from argos import argos_runner, argos_io

env = gym.make('ARGoS-v0')
env.set_argos(argos_runner.Argos(), argos_io.ArgosIO(num_robots=1, verbose=False))
env.reset()

#for i in range(100000):
    #env.render()
#    env.step(env.action_space.sample())

for i_episode in range(100):
    observation = env.reset()
    print("Episode: ", i_episode, " of ", 10000)
    for t in range(1000):
        env.render()
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if t % 10 == 0:
            print("Obs: ", observation)
        if observation <= 10:
            print("Episode finished after {} timesteps".format(t+1))
            break


