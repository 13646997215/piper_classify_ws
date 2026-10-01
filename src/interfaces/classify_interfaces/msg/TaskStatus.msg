std_msgs/Header header
string state
string stage
string current_object
string target_bin
uint8 completed
uint8 total
uint8 succeeded
uint8 failed
float64 gripper_open_ratio
string held_object
float64 elapsed_sec
string detail
# Released objects are owned by the task, not by pickup-area perception.
string[] placed_objects
geometry_msgs/Pose[] placed_poses
