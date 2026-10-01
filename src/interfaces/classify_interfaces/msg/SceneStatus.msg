std_msgs/Header header
bool ready
bool camera_stable
string[] fresh_colors
string[] stale_colors
string detail

# Parallel ID/color arrays; IDs are assigned by visual tracking, not Gazebo.
string[] object_ids
string[] object_colors
string[] fresh_objects
string[] stale_objects
