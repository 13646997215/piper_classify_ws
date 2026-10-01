std_msgs/Header header
bool success
uint8 succeeded
uint8 failed
float64 elapsed_sec
string detail
string[] objects
string[] bins
geometry_msgs/Pose[] final_poses

# Simulation verification binding, established only after physical grasp.
string[] physical_objects
