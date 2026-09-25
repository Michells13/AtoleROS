# Contenido de Config.xml aplanado (lo publica config_manager, latched).
std_msgs/Header header
uint32 version                     # sube en cada cambio
string path
diagnostic_msgs/KeyValue[] entries # clave = ruta XML (p. ej. Robot/Aubo/IP)
