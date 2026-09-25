# Salud del sistema (lo publica system_monitor).
std_msgs/Header header
string phase                       # INIT_CHECKS | WARMUP | READY | FAULT
bool ready                         # true cuando se puede lanzar una misión
HealthItem[] items
