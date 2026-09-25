"""dataset_player — modo SIM: publica los mismos topics que las cámaras desde un dataset.

Fase 0: esqueleto. Expone la interfaz definitiva; la lógica llega en la Fase 2.
"""
import rclpy
from rclpy.node import Node

from atole_common.stubs import run_node, stub_action, stub_service
from atole_interfaces.srv import ListDatasets, LoadDataset
from std_srvs.srv import Trigger

PHASE = 2


class DatasetPlayer(Node):

    def __init__(self):
        super().__init__('dataset_player')
        stub_service(self, ListDatasets, '/atole/sim/list', PHASE)
        stub_service(self, LoadDataset, '/atole/sim/load', PHASE)
        stub_service(self, Trigger, '/atole/sim/stop', PHASE)
        self.get_logger().info('dataset_player listo (esqueleto; lógica en la Fase 2)')


def main(args=None):
    run_node(DatasetPlayer, args)


if __name__ == '__main__':
    main()
