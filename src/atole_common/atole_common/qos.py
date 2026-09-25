"""Perfiles QoS comunes de AtoleROS."""
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)

# Topics de estado que un nodo que llega tarde debe recibir igualmente (config, salud, estado).
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

# Imágenes y nubes: solo interesa la última.
SENSOR = qos_profile_sensor_data
