Header header

nav_msgs/Odometry true_odom
geometry_msgs/Point last_true_pos
float64 last_true_heading
nav_msgs/Odometry ekf_odom
float64[6] last_ekf_state

bool has_camera_update
CameraUpdate camera_update

float64 prius_steer_angle
float64 prius_throttle_percent

PathUpdate path_update
