# Estado del brazo AUBO iS10 (lo publica arm_driver).
std_msgs/Header header             # frame_id = base_link
bool connected
float64[6] joints                  # rad, orden shoulder..wrist3
geometry_msgs/Pose tcp_pose        # TCP en base_link
float64[6] tcp_pose_native         # getTcpPose del AUBO: x y z roll pitch yaw (RPY intrínseco ZYX)
float64[6] tcp_offset_native       # getTcpOffset: brida -> TCP, misma convención
bool is_moving
bool freedrive
bool j6_locked
string robot_mode
string safety_mode
bool protective_stop
bool emergency_stop
string control_owner               # dueño actual del lease ("" = libre)
string last_fault
