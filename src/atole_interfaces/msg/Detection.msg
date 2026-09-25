int32 id                           # índice en orden de score descendente (no es estable entre frames)
string class_name
float32 score
int32[4] bbox_xywh                 # píxeles
int32[2] mask_xy                   # esquina superior izquierda de la máscara en la imagen
sensor_msgs/Image mask             # mono8 (0/1) recortada a la caja que envuelve máscara y bbox (vacía si no se pidió)
