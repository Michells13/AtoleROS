# Detecciones de una captura (las publica detector_node).
std_msgs/Header header             # stamp = stamp de la imagen; frame_id = marco óptico de la cámara
string camera                      # cam0 | cam1 | camC | cam2 | sim
string model_name
uint32 image_width
uint32 image_height
float32 inference_ms
Detection[] detections
