"""pod_selector — elige el pod (proximidad, izquierda→derecha…) y comprueba IK.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 4.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.srv import SelectPod

PHASE = 4


class PodSelector(Node):

    def __init__(self):
        super().__init__('pod_selector')
        stub_service(self, SelectPod, '/atole/task_planning/select_pod', PHASE)
        self.get_logger().info('pod_selector listo (esqueleto; lógica en la Fase 4)')


def main(args=None):
    run_node(PodSelector, args)


if __name__ == '__main__':
    main()
