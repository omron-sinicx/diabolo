# Diabolo play with robots

This repository holds the packages and scripts for making our UR5e robot arms play with diabolos.


### Structure

- `scripts` contains player nodes (to drive the robots), experiment treatment scripts and debugging tools. 
- `diabolo_motion_generator.cpp` contains the classes `DiaboloMotionGenerator` and `DiaboloPredictor`. The former generates a stick trajectory for a desired diabolo trajectory using a random walk. The latter is used to obtain the state of the diabolo at the next timestep, given the current diabolo state and stick positions.
- The robot drivers or simulation need to be started separately.

## Player nodes

You can send motion commands from different player nodes. The stick target positions (and other ) are visualized with VisualizationMarkers.

* `rosrun diabolo_play p_controller.py`

A very simple up and down motion that might accelerate a diabolo with a bearing.

* `rosrun diabolo_play interactive_play.py`

The main script, containing a number of ways to move the robots (continuous circular motion, throwing).

* `rosrun diabolo_play publish_2D_stick_sine_waves.py`

Not a player node, but a tool to evaluate if the motions look natural and IK works throughout the workspace.


## Using the interactive_play.py player node

If the node fails to start up, prepare the experiment data below.

- Enter `1` to load the saved motions (mandatory)
- Enter `2` to load the default settings
- Enter `3` to move to the starting position of the loaded motion
- Enter `d` to spawn the diabolo in simulation
- Enter `px`, `y` to start a periodic motion
- Enter `t` to stop


## Preparing/treating experiment data

Experiment data (both raw and treated) is in the **experiments** folder. The folder **experiments/recordings** contains raw data recorded during the experiments. **experiments/output** contains only data that can be generated from the recordings.

Recorded data needs to be filtered and offset, because the motion capture frames are not at the center of the diabolos or the tip of the sticks. In the `..._calib.csv` files, the diabolos are placed such that the center is above the ground plane origin, and the sticks are held such that the tip is at the origin.

To treat experiment data, use `data_preparation_recalib.py` in the `scripts` folder:

```
rosrun diabolo_play data_preparation_recalib.py 
```

E.g.:

```
rosrun diabolo_play data_preparation_recalib.py 2020-09-14_motion_extraction/linear_accel_stick_motion.csv
```

Press enter (unless your experiment file has a different column header, such as `diabolo_blue_2020_10`). Press `1`, `2` and `w` in order to load, convert and write the file to the `experiments/output` directory.

Alternatively:

1. Open the `Diabolo Data Extraction.ipynb` Jupyter Notebook 
2. Use the example in the second cell (after executing cells 3-6) to create 


## Visualizing experiment data

On a system with ROS (or in one of our containers), execute:

1. `roslaunch diabolo_play rviz.launch`

2. `rosrun diabolo_play playback_experiment.py`

You can use the command-line interface of the second node to play back the data in Rviz.
