import sys

import gym
import gym.spaces

import math
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class DiaboloEnv(gym.Env):
    def __init__(self, task_mode="position_control"):
        # action
        self.stick_pos_threshold = 2
        stick_pos_threshold = np.array([self.stick_pos_threshold * 2, np.finfo(np.float32).max,
                                        self.stick_pos_threshold * 2, np.finfo(np.float32).max,
                                        self.stick_pos_threshold * 2, np.finfo(np.float32).max])
        self.action_space = gym.spaces.Box(low=stick_pos_threshold, high=stick_pos_threshold)

        # state
        self.pitch_threshold = 1.5
        self.yaw_threshold = 1.5
        ori_threshold = np.array([self.pitch_threshold * 2, np.finfo(np.float32).max,
                                  self.yaw_threshold * 2, np.finfo(np.float32).max])
        self.observation_space = gym.spaces.Box(low=-ori_threshold, high=ori_threshold)

        # reset
        self.reset()

        # task mode
        if task_mode == "position_control" or task_mode == "orientation_control" or task_mode == "position_orientation_control":
            self.task_mode = task_mode
        else:
            self.task_mode = "position_control"
        print("Task mode is {}".format(self.task_mode))

        # figure
        self.fig = plt.figure()
        self.ax = Axes3D(self.fig)

    def reset(self):
        self.diabolo_pos = [0, 0, 0.5]
        self.diabolo_ori = [0, 0]
        self.right_stick_pos = [d + r for d, r in zip(self.diabolo_pos, [0, -0.15, 0.7])]
        self.left_stick_pos = [d + l for d, l in zip(self.diabolo_pos, [0, 0.15, 0.7])]
        self.done = False
        self.steps = 0
        return self.observe()

    def step(self, action):
        self.right_stick_pos = action[0:3]
        self.left_stick_pos = action[3:6]

        # TODO update diabolo state based on analytical model
        # input self.right_stick_pos and self.left_stick_pos
        # output self.diabolo_pos
        self.diabolo_pos[0] = (self.right_stick_pos[0] + self.left_stick_pos[0]) / 2.0
        self.diabolo_pos[1] = (self.right_stick_pos[1] + self.left_stick_pos[1]) / 2.0
        self.diabolo_pos[2] = (self.right_stick_pos[2] + self.left_stick_pos[2]) / 2.0 - 0.7

        diabolo_pos_and_ori = self.observe()
        reward = self.get_reward(diabolo_pos_and_ori)
        self.done = self.is_done(diabolo_pos_and_ori)
        return diabolo_pos_and_ori, reward, self.done, {}

    def render(self, close=False):
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.set_zlim(0, 2)

        self.ax.set_aspect('equal')

        # right stick
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = 0.03 * np.outer(np.cos(u), np.sin(v)) + self.right_stick_pos[0]
        y = 0.03 * np.outer(np.sin(u), np.sin(v)) + self.right_stick_pos[1]
        z = 0.03 * np.outer(np.ones(np.size(u)), np.cos(v)) + self.right_stick_pos[2]
        self.ax.plot_surface(x, y, z, color="green")

        # left stick
        u = np.linspace(0, 2 * np.pi, 10)
        v = np.linspace(0, np.pi, 10)
        x = 0.03 * np.outer(np.cos(u), np.sin(v)) + self.left_stick_pos[0]
        y = 0.03 * np.outer(np.sin(u), np.sin(v)) + self.left_stick_pos[1]
        z = 0.03 * np.outer(np.ones(np.size(u)), np.cos(v)) + self.left_stick_pos[2]
        self.ax.plot_surface(x, y, z, color="green")

        # diabolo
        pitch = self.diabolo_ori[0]
        yaw = self.diabolo_ori[1]
        forward_x = 0.1 * math.cos(pitch) * math.cos(yaw)
        forward_y = 0.1 * math.cos(pitch) * math.sin(yaw)
        forward_z = 0.1 * math.sin(pitch)
        x = np.array([forward_x + self.diabolo_pos[0], -forward_x + self.diabolo_pos[0]])
        y = np.array([forward_y + self.diabolo_pos[1], -forward_y + self.diabolo_pos[1]])
        z = np.array([forward_z + self.diabolo_pos[2], -forward_z + self.diabolo_pos[2]])
        self.ax.plot(x, y, z, color="blue")

        # right string
        x = np.array([self.right_stick_pos[0], self.diabolo_pos[0]])
        y = np.array([self.right_stick_pos[1], self.diabolo_pos[1]])
        z = np.array([self.right_stick_pos[2], self.diabolo_pos[2]])
        self.ax.plot(x, y, z, color="black")

        # left string
        x = np.array([self.left_stick_pos[0], self.diabolo_pos[0]])
        y = np.array([self.left_stick_pos[1], self.diabolo_pos[1]])
        z = np.array([self.left_stick_pos[2], self.diabolo_pos[2]])
        self.ax.plot(x, y, z, color="black")

        # plot
        plt.draw()
        plt.pause(0.0001)
        plt.cla()

    def is_done(self, diabolo_pos_and_ori):
        done = False
        diabolo_pos = diabolo_pos_and_ori[0:3]
        diabolo_ori = diabolo_pos_and_ori[3:5]

        if self.task_mode == "orientation_control":
            done = bool(diabolo_ori[0] < -self.pitch_threshold or diabolo_ori[0] > self.pitch_threshold or diabolo_ori[1] < -self.yaw_threshold or diabolo_ori[1] > self.yaw_threshold)

        return done

    def get_reward(self, diabolo_pos_and_ori):
        reward = 0.0
        done = self.is_done(diabolo_pos_and_ori)
        if not done:
            reward = 1.0
        return reward

    def observe(self):
        diabolo_pos_and_ori = np.array(self.diabolo_pos + self.diabolo_ori)
        return diabolo_pos_and_ori
