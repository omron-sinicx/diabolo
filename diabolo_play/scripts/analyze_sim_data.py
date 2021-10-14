#!/usr/bin/env python
import rospkg
import rospy
import pandas as pd
import sys
import os
from matplotlib import pyplot as plt
import numpy as np
import math

# This file is meant to fit simulation parameters to recorded experiment data.

class SimOutcomePlotter:
    def __init__(self):
        self._rospack = rospkg.RosPack()
        self._package_directory = self._rospack.get_path("diabolo_play")
        self.output_directory = os.path.join(
            self._package_directory, "experiments", "plugin_outputs"
        )

    def set_trial_name(self, num):
        self.trial_name = "trial_" + str(num)
        print("The trial name has been set to " + self.trial_name)
        self.data_directory = os.path.join(self.output_directory, self.trial_name)

    def read_data(self, file):
        df = pd.read_csv(file, header=[0, 1, 2])
        # Setting the time from t=0
        df.iloc[:, 0][:] -= df.iloc[:, 0][0]
        return df

    def plot_graph(self):
        plt.plot(self.df.iloc[:, 0][:], self.errors)
        plt.show()

    def get_position_error(self, df):
        pl_pos = np.array(df["plugin"]["position"])
        exp_pos = np.array(df["experiment"]["position"])
        errors = np.ones(pl_pos.shape[0])
        for i in range(pl_pos.shape[0]):
            errors[i] = np.linalg.norm(pl_pos[i] - exp_pos[i])
        return errors

    def calculate_ideal_parameters(self):
        experiments = [
            d
            for d in os.listdir(self.data_directory)
            if os.path.isdir(os.path.join(self.data_directory, d))
        ]
        self.exp_errors = {d: 1.0 for d in experiments}
        for exp in experiments:
            files = os.listdir(os.path.join(self.data_directory, exp))
            param_list = [
                f.replace(exp + "_", "").replace(".csv", "").split("_") for f in files
            ]
            print(str(len(param_list)) + " data files found for " + exp)

            min_error = float("inf")  # Arbitrarily large number
            max_error = float("-inf")
            both_errors = {
                "min_error": min_error,
                "min_error_params": (1, 1, 1, 1),
                "max_error": max_error,
                "max_error_params": (1, 1, 1, 1),
            }
            for i in range(len(files)):
                df = self.read_data(os.path.join(self.data_directory, exp, files[i]))
                errors = self.get_position_error(df)
                avg_error = np.sum(errors) / errors.shape[0]
                if avg_error < min_error:
                    both_errors["min_error"] = avg_error
                    both_errors["min_error_params"] = tuple(param_list[i])
                    min_error = avg_error
                if avg_error > max_error:
                    both_errors["max_error"] = avg_error
                    both_errors["max_error_params"] = tuple(param_list[i])
                    max_error = avg_error

            # Set the dict entry to the lowest error and the parameters giving that error
            self.exp_errors[exp] = both_errors


if __name__ == "__main__":
    rospy.init_node("sim_data_analyzer", anonymous=True)
    s = SimOutcomePlotter()
    print("Enter the trial number")
    i = raw_input()
    s.set_trial_name(i)

    while i and not rospy.is_shutdown():
        # rospy.loginfo("Enter p to plot data for a particular experiment file")
        rospy.loginfo("Enter c to calculate the best parameters for a trial")
        rospy.loginfo("Enter n to set new trial name")
        rospy.loginfo("Enter x to exit.")

        i = raw_input()
        if i == "p" or i == "P":
            print("Enter the name of the experiment data file")
            filename = raw_input()
            print("Plotting data for " + filename + " experiment")
            p = []
            while len(p) < 4:
                print("Enter 4 plugin parameters, seperated by spaces")
                p = raw_input().split()
                # TODO: Add code to plot an error graph for an experiment and some parameters
        elif i == "c" or i == "C":
            s.calculate_ideal_parameters()
            for k, v in s.exp_errors.iteritems():
                print("For exp " + str(k))
                print(
                    "Min error "
                    + str(v["min_error"])
                    + " occurs at "
                    + str(v["min_error_params"])
                )
                print(
                    "Max error "
                    + str(v["max_error"])
                    + " occurs at "
                    + str(v["max_error_params"])
                )
        elif i == "n" or i == "N":
            print("Enter the new trial number")
            i = raw_input()
            s.set_trial_name(i)
        elif i == "x" or "X":
            break
