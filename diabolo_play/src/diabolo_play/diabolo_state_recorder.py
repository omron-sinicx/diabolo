#!/usr/bin/env python
#Class file containing DiaboloSimRecorder and DiaboloSimTrial classes 
#DiaboloSimRecorder: Records the
# ,compare the relative poses and rotation velocities and write the difference and error scores to a csv file
# The script will use the latest version of the diabolo pose and velocity for both experiment (real) diabolo and plugin (simulated) diabolo

import csv
from diabolo_gazebo.msg import DiaboloState
from std_msgs.msg import String
from tf.transformations import *
import rospy
import os
import math
import rospkg
import numpy as np
class DiaboloSimTrial():
    #Store a list of diabolo states for a certain experiment
    def __init__(self):
        self.plugin_state_list = [] #list of simulated diabolo states
        self.exp_state_list = [] #list of experiement (real) diabolo states
        self.plugin_state_topic = "/diabolo_gazebo_node/diabolo_current_state"
        self.latest_plugin_state = None
        self.plugin_diabolo_state_sub = rospy.Subscriber(self.plugin_state_topic, DiaboloState, self.plugin_state_callback)
        self.sim_parameters = []
        self.experiment_name = ""
        print("Started new DiaboloSimTrial")

    #Store the latest plugin state msg
    def plugin_state_callback(self, msg):
        self.latest_plugin_state = msg

    #Needs a DiaboloState msg with the current experiement data
    #Store both experiment state and plugin state at once to ensure the same number of data points for both messages 
    def store_diabolo_states(self, exp_diabolo_state): 
        if self.latest_plugin_state: #Only store data if DiaboloState has been received from plugin
            self.plugin_state_list.append(self.latest_plugin_state)
            self.exp_state_list.append(exp_diabolo_state)

    def stop_recording(self):
        self.plugin_diabolo_state_sub.unregister()
            
class DiaboloSimRecorder():
    #Evaluate the error score given a list of simulated diabolo states and a list of experiment (real) diabolo states
    # 
    def __init__(self, filename="default.csv"):
        self._rospack = rospkg.RosPack()
        self._package_directory = self._rospack.get_path('diabolo_play')
        self.exp_state_topic = "/experiment_diabolo_state"
        self.exp_data_available = False
        self.plugin_data_available = False # Check if any messages have been received
        self.outcome_list = [] #A list to hold all experiment outcomes
        self.current_trial = None # The current outcome that stores the plugin states and the experiment states
        self.exp_name = filename.split('.')[0]
        print("Started new DiaboloSimRecorder")

    def record_data(self):
        # Records the data of the current outcome to a data file
        directory = os.path.join(self._package_directory, "experiments", "plugin_outputs")
        if not os.path.exists(directory):
            os.makedirs(directory)
        pv_pre_cap = self.sim_parameters_[0]
        pv_cap = self.sim_parameters_[1]
        pv_post_cap = self.sim_parameters_[2]
        constrain_scale = self.sim_parameters_[3]
        caps = "_" + str(pv_pre_cap) + "_" + str(pv_cap) + "_" +str(pv_post_cap) + "_" + str(constrain_scale)

        file = open(os.path.join(directory, self.exp_name + caps +".csv"), 'w')
        file.write(",plugin,plugin,plugin,plugin,plugin,plugin,plugin,experiment,experiment,experiment,experiment,experiment,experiment,experiment,,,plugin,plugin,plugin,,,")
        file.write("\n")
        file.write(",position,position,position,orientation,orientation,orientation,orientation,position,position,position,orientation,orientation,orientation,orientation,,,trans_vel,trans_vel,trans_vel,,,")
        file.write("\n")
        file.write("time,X,Y,Z,X,Y,Z,W,X,Y,Z,X,Y,Z,W,p_rot_vel,e_rot_vel,X,Y,Z,mass,string_length,step_wise_error")
        file.write("\n")
        print("File name is " + os.path.join(directory, self.exp_name + caps +".csv"))

        for i in range(len(self.current_trial.plugin_state_list)):
            p_pos = np.array([self.current_trial.plugin_state_list[i].pose.position.x,
                                self.current_trial.plugin_state_list[i].pose.position.y,
                                self.current_trial.plugin_state_list[i].pose.position.z])

            e_pos = np.array([self.current_trial.exp_state_list[i].pose.position.x,
                                self.current_trial.exp_state_list[i].pose.position.y,
                                self.current_trial.exp_state_list[i].pose.position.z])

            p_rot = np.array([self.current_trial.plugin_state_list[i].pose.orientation.x,
                                self.current_trial.plugin_state_list[i].pose.orientation.y,
                                self.current_trial.plugin_state_list[i].pose.orientation.z,
                                self.current_trial.plugin_state_list[i].pose.orientation.w])

            e_rot = np.array([self.current_trial.exp_state_list[i].pose.orientation.x,
                                self.current_trial.exp_state_list[i].pose.orientation.y,
                                self.current_trial.exp_state_list[i].pose.orientation.z,
                                self.current_trial.exp_state_list[i].pose.orientation.w])
            p_vel = np.array([self.current_trial.plugin_state_list[i].trans_velocity.x,
                                self.current_trial.plugin_state_list[i].trans_velocity.y,
                                self.current_trial.plugin_state_list[i].trans_velocity.z])

            p_rot_vel = self.current_trial.plugin_state_list[i].rot_velocity
            e_rot_vel = self.current_trial.exp_state_list[i].rot_velocity

            step_wise_error = self._calculate_stepwise_error(p_pos, e_pos, p_rot, e_rot, p_rot_vel, e_rot_vel)

            time = [str(self.current_trial.plugin_state_list[i].header.stamp.secs + self.current_trial.plugin_state_list[i].header.stamp.nsecs * 1e-9)]
            p_pos_str = [str(pp) for pp in p_pos]
            p_rot_str = [str(pr) for pr in p_rot]
            e_pos_str = [str(ep) for ep in e_pos]
            e_rot_str = [str(er) for er in e_rot]
            p_rot_vel_str = [str(self.current_trial.plugin_state_list[i].rot_velocity)]
            e_rot_vel_str = [str(self.current_trial.plugin_state_list[i].rot_velocity)]
            p_vel_str = [str(pv) for pv in p_vel]
            p_mass_str = [str(self.current_trial.plugin_state_list[i].mass)]
            p_string_length_str = [str(self.current_trial.plugin_state_list[i].string_length)]
            step_wise_error_str = [str(step_wise_error)]

            data = time + p_pos_str + p_rot_str + e_pos_str + e_rot_str + p_rot_vel_str + e_rot_vel_str + p_vel_str + p_mass_str + p_string_length_str + step_wise_error_str

            file.write(",".join(data))
            file.write("\n")
        
        file.close()

    #Calculate the error at each time step, and return it as a list
    def _calculate_stepwise_error(self, p_pos, e_pos, p_rot, e_rot, p_rot_vel, e_rot_vel):
        #TODO: Define a reasonable error function. Taking sum of euclidian norm for transformation and relative rotation for now
        pos_error_val = np.linalg.norm(p_pos - e_pos)

        p_rot_inv = p_rot
        p_rot_inv[3] = -p_rot_inv[3]
        rot_error = quaternion_multiply(e_rot, p_rot_inv) #Get relative rotation
        rot_error_val = np.linalg.norm(p_rot - e_rot)

        rot_vel_error_val = abs(p_rot_vel - e_rot_vel)

        return pos_error_val + rot_error_val + rot_vel_error_val

    def end_recording_trial(self):
        self.current_trial.stop_recording()
        self.record_data() #Write data in trial to data file

        self.outcome_list.append(copy.deepcopy(self.current_trial))
        
    #Start storing the simulation outcome data for a given configuration
    #Parameters: 
    # exp_name => The name of the experiment (ground-truth) file being read in
    # parameters => The a tuple containing the simulation parameters (Assumed to be the pull velocity scaling parameters)
    #               Order -> pv_pre_cap_scaling_factor, pv_cap_scaling_factor, pv_post_cap_scaling_factor 
    def start_recording_trial(self, sim_parameters):
        self.sim_parameters_ = list(sim_parameters)
        self.current_trial = DiaboloSimTrial()
        self.current_trial.sim_parameters = self.sim_parameters_
        self.current_trial.experiment_name = self.exp_name
    #Store the incoming exp diabolo state as well as the latest available plugin diabolo state
    #This function must be called every time a new exp diabolo state is published
    def store_data_point(self, exp_diabolo_state): 
        self.current_trial.store_diabolo_states(exp_diabolo_state)


    



    
