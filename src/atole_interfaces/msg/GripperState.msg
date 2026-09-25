# Estado del gripper RGI-100 (lo publica grip_twist_controller).
std_msgs/Header header
bool connected
string port
int8 init_state                    # 0 sin inicializar, 1 listo, 2 inicializando
int8 gripper_state                 # 0 moviendo, 1 llegó, 2 agarró objeto, 3 objeto caído
string init_state_name
string gripper_state_name
bool busy
int32 last_opening                 # 0..1000 (1000 = abierto)
int32 last_angle_deg
