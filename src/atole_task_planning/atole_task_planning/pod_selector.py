"""pod_selector — ordena los pods según una estrategia y comprueba que se pueden alcanzar.

/atole/task_planning/select_pod (SelectPod):
  1. Candidatos: pods con status "ok" (con pose y dentro del área de trabajo).
  2. Alcanzabilidad: IK de frente (arm_driver compute_ik, prefer_facing) del pre-pick y del pick
     calculados como los ejecutará mission_manager (atole_common.grasp). Si falla, el pod queda
     reachable = false y status "unreachable: <motivo>".
  3. Orden (Selection/Strategy o la petición), en base_link (X adelante, Y izquierda, Z arriba):
       proximity      punto de agarre más cercano al origen de base_link
       left_to_right  Y de mayor a menor
       right_to_left  Y de menor a mayor
       confidence     score de detección × confianza de la pose, de mayor a menor
  selected_index = primer pod alcanzable en ese orden (−1 si no hay ninguno).
"""
import numpy as np
from atole_interfaces.srv import ComputeIk, SelectPod
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from atole_common import grasp
from atole_common.config_view import ConfigView
from atole_common.stubs import run_node
from atole_common.sync_calls import CallError, call

STRATEGIES = {
    'proximity': lambda p, g: float(np.linalg.norm(g)),
    'left_to_right': lambda p, g: -g[1],
    'right_to_left': lambda p, g: g[1],
    'confidence': lambda p, g: -(p.score * p.pose_confidence),
}


class PodSelector(Node):

    def __init__(self):
        super().__init__('pod_selector')
        group = ReentrantCallbackGroup()
        self.config = ConfigView(self)
        self.ik = self.create_client(ComputeIk, '/atole/arm/compute_ik', callback_group=group)
        self.create_service(SelectPod, '/atole/task_planning/select_pod', self._srv_select, callback_group=group)
        self.get_logger().info('pod_selector listo')

    def _reachable(self, pod, harvest):
        t = grasp.targets(pod, harvest)
        for name in ('pre', 'pick'):
            pos, quat = t[name]
            res = call(self.ik, ComputeIk.Request(target=grasp.pose_stamped(pos, quat), prefer_facing=True), timeout=5.0)
            if not res.ok:
                return False, f'sin IK para el {"pre-pick" if name == "pre" else "pick"} ({res.message})'
        return True, ''

    def _srv_select(self, req, res):
        if not self.config.wait_ready():
            res.ok, res.message, res.selected_index = False, 'sin /atole/config', -1
            return res
        strategy = req.strategy or self.config.get('Selection/Strategy', 'proximity')
        if strategy not in STRATEGIES:
            res.ok, res.message, res.selected_index = False, f'estrategia desconocida "{strategy}" ({", ".join(STRATEGIES)})', -1
            return res
        harvest = self.config.section('Harvest')
        pods = list(req.pods.pods)
        for pod in pods:
            pod.reachable = False
            if pod.status != 'ok':
                continue
            try:
                pod.reachable, why = self._reachable(pod, harvest)
            except CallError as e:
                res.ok, res.message, res.selected_index = False, f'arm_driver no responde: {e}', -1
                return res
            if not pod.reachable:
                pod.status = f'unreachable: {why}'
        key = STRATEGIES[strategy]
        point = lambda p: grasp.targets(p, harvest)['pick'][0]
        res.order = sorted(range(len(pods)), key=lambda i: (not pods[i].reachable, key(pods[i], point(pods[i]))))
        res.selected_index = next((i for i in res.order if pods[i].reachable), -1)
        res.evaluated = req.pods
        res.evaluated.pods = pods
        n_ok = sum(p.reachable for p in pods)
        res.ok = True
        res.message = (f'{strategy}: {n_ok}/{len(pods)} alcanzables; '
                       + (f'elegido pod {pods[res.selected_index].id}' if res.selected_index >= 0 else 'ninguno alcanzable'))
        self.get_logger().info(res.message)
        return res


def main(args=None):
    run_node(PodSelector, args)


if __name__ == '__main__':
    main()
