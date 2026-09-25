"""Utilidades para los nodos esqueleto y el arranque de nodos."""
import signal

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor


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


def run_node(node_class, args=None, single_threaded=False):
    """Arranca un nodo con executor multihilo (o de un hilo) y cierre limpio con Ctrl+C.
    single_threaded: para nodos que crean y destruyen suscripciones desde sus callbacks; con el
    executor multihilo de rclpy eso puede romper el wait set (InvalidHandle en event_handler)."""
    rclpy.init(args=args)
    node = node_class()
    executor = SingleThreadedExecutor() if single_threaded else MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C llega dos veces (al grupo del terminal y reenviado por launch): un segundo SIGINT
        # a mitad del cierre mataría el nodo (p. ej. camera_manager dejaría huérfanas las ZED).
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
