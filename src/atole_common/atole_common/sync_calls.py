"""Llamadas síncronas a servicios y acciones desde callbacks (con MultiThreadedExecutor y un grupo
reentrante): esperan con threading.Event en vez de hacer spin, y las acciones se pueden cancelar."""
import threading


class CallError(RuntimeError):
    pass


def call(client, request, timeout=10.0):
    """Respuesta del servicio o CallError si no está o no responde a tiempo."""
    if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
        raise CallError(f'servicio {client.srv_name} no disponible')
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        raise CallError(f'{client.srv_name} sin respuesta en {timeout:.0f} s')
    return future.result()


def run_action(client, goal, timeout, feedback=None, cancel=None, on_goal=None):
    """Envía un goal y espera el resultado. `cancel` (threading.Event) lo cancela; `on_goal(handle)`
    recibe el handle en cuanto se acepta. Devuelve el Result o lanza CallError."""
    if not client.wait_for_server(timeout_sec=5.0):
        raise CallError(f'acción {client._action_name} no disponible')
    accepted = threading.Event()
    sent = client.send_goal_async(goal, feedback_callback=(lambda f: feedback(f.feedback)) if feedback else None)
    sent.add_done_callback(lambda _f: accepted.set())
    if not accepted.wait(10.0):
        raise CallError(f'{client._action_name}: el goal no fue aceptado a tiempo')
    handle = sent.result()
    if not handle.accepted:
        raise CallError(f'{client._action_name}: goal rechazado (¿ocupado?)')
    if on_goal:
        on_goal(handle)
    finished = threading.Event()
    result = handle.get_result_async()
    result.add_done_callback(lambda _f: finished.set())
    waited = 0.0
    while not finished.wait(0.1):
        waited += 0.1
        if cancel is not None and cancel.is_set():
            handle.cancel_goal_async()
            finished.wait(10.0)
            raise CallError('cancelado')
        if waited > timeout:
            handle.cancel_goal_async()
            raise CallError(f'{client._action_name} sin resultado en {timeout:.0f} s')
    return result.result().result
