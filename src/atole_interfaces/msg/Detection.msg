int32 id
string class_name
float32 score
int32[4] bbox_xywh                 # píxeles
sensor_msgs/Image mask             # mono8, mismo tamaño que la imagen (vacía si no se pidió)
