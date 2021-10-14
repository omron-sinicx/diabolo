geometry_msgs/PoseArray initial_poses # Initial positions of diabolo [0], stick_left [1] and stick_right[2]
float32 rot_velocity # Rotational velocity of the diabolo
geometry_msgs/Point trans_velocity # Translation velocity of the diabolo (world frame)
float32 pv_pre_cap_scale # pv = pull velocity
float32 pv_post_cap_scale
float32 pv_cap_scale
float32 velocity_diffusion_factor # The factor by which the velocity reduces every time step
float32 time_step # The length of a time step
bool caps_valid # Set to true if want to change the pv scaling factors
float32 mass # Mass of the diabolo. Ignored if 0
float32 axle_radius # Radius of the diabolo axle
float32 string_length # Ignored if 0 or less than initial distance between strings
string file_name # This is the name of the experiment file. 
