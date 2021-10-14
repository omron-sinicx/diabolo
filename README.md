# Playing diabolo with robots

This repository holds the ROS packages for playing diabolo with two UR5e robot arms on ROS Melodic (Ubuntu 18.04).

Read our blog post [here](https://medium.com/sinicx/an-analytical-diabolo-model-for-robotic-learning-and-control-c8d8a8384d6d) or watch our ICRA presentation [here](https://www.youtube.com/watch?time_continue=1&v=oS-9mCfKIeY&feature=emb_logo).

With this package, you can:

- Spin the diabolo (linear acceleration)

![Linear acceleration](https://github.com/omron-sinicx/diabolo/wiki/linear.gif)

- Spin the diabolo in a circle (circular acceleration)
- Throw the diabolo in different directions

![Acceleration and throwing](https://github.com/omron-sinicx/diabolo/wiki/multiview.gif)

- Pass the diabolo to yourself and have the robot catch it (if your throw was good)

![Passing](https://github.com/omron-sinicx/diabolo/wiki/passing.gif)

- Simulate the diabolo in Gazebo and play with it

![Simulation](https://github.com/omron-sinicx/diabolo/wiki/simulation.gif)

Train your robot to play with the diabolo or set different goals manually to change the motions.


## QUICKSTART

Install [ignition-math6](https://ignitionrobotics.org/api/math/6.2/install.html).

1.  Start up the robots and drivers. Either in simulation:

    `roslaunch diabolo_gazebo diabolo_gazebo.launch`

    Or in the real world:

    `roslaunch diabolo_scene_description connect_real_robots.launch`

2.  Then, in separate terminals:
    
    `roslaunch diabolo_moveit_config diabolo_moveit_planning_execution.launch`  
    `rosrun diabolo_play stick_target_to_joint_target_converter`
    `rosrun diabolo_play diabolo_motion_generator`
    
    Note the visualization options in the Displays panel on the left.

3.  Spawn a diabolo in the scene and move the robots with a player node:

    `rosrun diabolo_play interactive_play.py`

    Enter the commands (pressing Enter after each): `1`, `2`, `3`, `d` (then `n`), `px` to start the circular motion.

    See the `diabolo_play` [package README](diabolo-play/README.md) for more options and player/testing nodes.

## More documentation

- [diabolo_play](diabolo_play/README.md)
- [diabolo_gazebo](diabolo_gazebo/README.md)

A dataset with over 40 minutes of human diabolo play is available [here](https://github.com/omron-sinicx/diabolo-datasets). The data can be reproduced with the scripts in the `diabolo_play` package.

## References

- "An analytical diabolo model for robotic learning and control", von Drigalski, F., Joshi, D., Murooka, T., Tanaka, K., Hamaya, M. and Ijiri, Y., ICRA 2021 [arXiv PDF](https://arxiv.org/abs/2011.09068)

- "Diabolo Orientation Stabilization by Learning Predictive Model for Unstable Unknown-Dynamics Juggling Manipulation", Murooka, T., Okada, K. and Inaba, M., IROS 2020 [PDF](http://ras.papercept.net/images/temp/IROS/files/3245.pdf)

- [bio_ik](https://github.com/TAMS-Group/bio_ik/) 

