import diabolo_gym
import gym

import numpy as np
import random
import sys, signal

def random_val(min_val, max_val):
    return random.randint(0, 1000) / 1000.0 * (max_val - min_val) + min_val

def policy(observation):
    right_stick_pos = [0, 0, 0]
    left_stick_pos = [0, 0, 0]

    diabolo_pos = observation[0:3]
    diabolo_ori = observation[3:5]
    right_stick_pos = [random_val(-0.2, 0.2), random_val(-0.2, 0), random_val(1.0, 1.5)]
    left_stick_pos = [random_val(-0.2, 0.2), random_val(0.2, 0), random_val(1.0, 1.5)]

    return np.array(right_stick_pos + left_stick_pos)

def test():
    env = gym.make('diabolo-v0')
    max_steps = 1000

    for episode in range(10):
        print("Episode: {}".format(episode))
        # reset at the start of episode
        observation = env.reset()
        episode_steps = 0
        episode_reward = 0.

        # start episode
        done = False
        while not done:
            action = policy(observation)

            observation, reward, done, info = env.step(action)
            if episode_steps > max_steps:
                done = True

            env.render()

            episode_reward += reward
            episode_steps += 1

if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda x, y: sys.exit(0))
    test()
