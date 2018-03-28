import gym
import gym_argos
from argos import argos_runner, argos_io
import sys
# initialize and run argos

num_robots = 15
argos = argos_runner.Argos()
argos_io = argos_io.ArgosIO(num_robots, verbose=False)
environments = []

for i in range(num_robots):
    env = gym.make('ARGoS-v0')
    env.set_argos(argos_io, i)
    env.reset()
    environments.append(env)

for i_episode in range(100):
    observation = env.reset()
    print("Episode: ", i_episode, " of ", 10000)
    for t in range(1000):
        for i in range(num_robots):
            action = environments[i].action_space.sample()
            observation, reward, done, info = environments[i].step(action)
            if t % 100 == 0:
                print("[", i,"]: ", reward, ", ", observation)
            #if observation <= 10:
            #    print("Episode finished after {} timesteps".format(t+1))
            #    break


