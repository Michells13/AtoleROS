"""Utilidades para los nodos esqueleto y el arranque de nodos."""
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor


def not_implemented(phase):
    return f'no implementado todavía (Fase {phase})'


def _fill(response, text):
    """Marca una respuesta como fallida rellenando los campos habituales."""
    for field in ('ok', 'success'):
        if hasattr(response, field):
            setattr(response, field, False)
    for field in ('message', 'msg'):
        if hasattr(response, field):
            setattr(response, field, text)
    return response


def stub_service(node, srv_type, name, phase):
    def callback(request, response):
        node.get_logger().warn(f'{name}: {not_implemented(phase)}')
        return _fill(response, not_implemented(phase))
    return node.create_service(srv_type, name, callback)


def stub_action(node, action_type, name, phase):
    def execute(goal_handle):
        node.get_logger().warn(f'{name}: {not_implemented(phase)}')
        goal_handle.abort()
        return _fill(action_type.Result(), not_implemented(phase))
    return ActionServer(node, action_type, name, execute_callback=execute)


def run_node(node_class, args=None):
    """Arranca un nodo con executor multihilo y cierre limpio con Ctrl+C."""
    rclpy.init(args=args)
    node = node_class()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
