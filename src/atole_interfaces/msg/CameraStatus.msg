# Estado de las cámaras (lo publica camera_manager).
std_msgs/Header header
string source                      # live | sim
string active_eth                  # cam0 | cam1 | camC
bool swap_in_progress
CameraSlot[] cameras
