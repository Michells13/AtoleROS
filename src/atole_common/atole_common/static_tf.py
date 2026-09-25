"""Broadcaster de /tf_static que sí actualiza transformaciones ya enviadas.

En rclpy (Jazzy) StaticTransformBroadcaster.sendTransform solo añade los child_frame_id nuevos: si
ya se envió uno, lo ignora y vuelve a publicar el valor viejo, sin error. Aquí se reemplaza antes el
de mismo hijo, así que recalibrar, cambiar el offset de herramienta o cargar otra vista SIM funciona.
"""
from tf2_ros import StaticTransformBroadcaster


class StaticTf(StaticTransformBroadcaster):

    def sendTransform(self, transform):
        transforms = transform if isinstance(transform, list) else [transform]
        for t in transforms:
            for i, old in enumerate(self.net_message.transforms):
                if old.child_frame_id == t.child_frame_id:
                    self.net_message.transforms[i] = t
                    break
        super().sendTransform(transforms)
