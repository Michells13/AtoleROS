"""Prueba de la misión en SIM con la VM del AUBO (sin gripper salvo --gripper).

Uso (AtoleROS lanzado con sim:=true robot:=vm, entorno ROS cargado):
    python3 tools/tests/test_fase4_mission.py [carpeta] [--all] [--step] [--abort-at ESTADO] [--gripper]
        [--eih none|single|fused_2view|ppp] [--eih-view VISTA] [--gate M] [--dry]

- Carga la carpeta del dataset, lanza /atole/mission/harvest y registra la secuencia de estados.
- Al salir de PREPICK y de PICK compara el TCP real con el objetivo que calcula atole_common.grasp
  para el pod elegido (debe coincidir en mm).
- --step confirma cada paso con /atole/mission/confirm_step; --abort-at aborta al entrar en ese estado.
- --eih elige la estrategia EiH (por defecto none); --eih-view carga esa vista real de cam2
  (/atole/sim/load_eih); --gate cambia EIH/CorrelateGateM sin guardarlo (la vista EiH y el dataset
  EtH son de escenas distintas); --dry = eih_dry_run.
"""
import sys
import threading
import time

import numpy as np
import rclpy
from atole_interfaces.action import Harvest
from atole_interfaces.msg import ArmState, MissionStatus, PodArray, SystemHealth
from atole_interfaces.srv import ConfigSet, LoadDataset
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from atole_common import grasp
from atole_common.config_view import ConfigView

args = sys.argv[1:]
opt = lambda name, default=None: args[args.index(name) + 1] if name in args else default
VALUES = {opt(n) for n in ('--abort-at', '--eih', '--eih-view', '--gate')}
FOLDER = next((a for a in args if not a.startswith('--') and a not in VALUES), '20260422_144848_954')
ABORT_AT = opt('--abort-at')
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


def main():
    rclpy.init()
    node = rclpy.create_node('fase4_test')
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    config = ConfigView(node)
    seen, checks, eih_log, data = [], [], [], {'arm': None, 'pods': None, 'status': None}
    node.create_subscription(ArmState, '/atole/arm/state', lambda m: data.__setitem__('arm', m), LATCHED)
    node.create_subscription(PodArray, '/atole/perception/pods', lambda m: data.__setitem__('pods', m), 10)
    confirm = node.create_client(Trigger, '/atole/mission/confirm_step')
    abort = node.create_client(Trigger, '/atole/mission/abort')

    def tcp():
        p = data['arm'].tcp_pose
        return np.array([p.position.x, p.position.y, p.position.z]), np.array(
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def compare(label, target):
        pos, quat = tcp()
        dq = abs(float(np.dot(quat / np.linalg.norm(quat), target[1])))
        ang = np.degrees(2 * np.arccos(min(1.0, dq)))
        checks.append(f'{label}: Δpos {1000 * np.linalg.norm(pos - target[0]):.2f} mm · Δori {ang:.2f}°')

    def on_status(m):
        prev = data['status']
        data['status'] = m
        if m.state == 'EIH_REFINE' and (prev is None or m.message != prev.message):
            eih_log.append(m.message)
        if prev is None or m.state != prev.state:
            seen.append(m.state)
            pods = data['pods']
            if prev is not None and pods and m.current_pod_id >= 0 and prev.state in ('PREPICK', 'PICK'):
                pod = next(p for p in pods.pods if p.id == m.current_pod_id)
                t = grasp.targets(pod, config.section('Harvest'))
                compare(prev.state, t['pre'] if prev.state == 'PREPICK' else t['pick'])
            if ABORT_AT and m.state == ABORT_AT:
                abort.call_async(Trigger.Request())
        if m.awaiting_confirmation and '--step' in args and not (prev and prev.awaiting_confirmation):
            print(f'   confirmo: {m.confirmation_prompt}')
            confirm.call_async(Trigger.Request())
    node.create_subscription(MissionStatus, '/atole/mission/status', on_status, LATCHED)

    load = node.create_client(LoadDataset, '/atole/sim/load')
    load.wait_for_service(timeout_sec=10)
    r = load.call(LoadDataset.Request(folder=FOLDER, rate_hz=2.0))
    print('dataset:', r.message)
    if opt('--eih-view'):
        load_eih = node.create_client(LoadDataset, '/atole/sim/load_eih')
        load_eih.wait_for_service(timeout_sec=10)
        print('vista EiH:', load_eih.call(LoadDataset.Request(folder=opt('--eih-view'), rate_hz=2.0)).message)
    if opt('--gate'):
        cset = node.create_client(ConfigSet, '/atole/config/set')
        cset.wait_for_service(timeout_sec=10)
        print('gate EiH:', cset.call(ConfigSet.Request(key='EIH/CorrelateGateM', value=opt('--gate'), persist=False)).message)
    health = {}
    node.create_subscription(SystemHealth, '/atole/system/health', lambda m: health.__setitem__('h', m), LATCHED)
    t_ready = time.monotonic()
    while not (health.get('h') and health['h'].ready):       # el dataset tarda unos segundos en verse
        if time.monotonic() - t_ready > 30:
            sys.exit(f'el sistema no llega a READY: {[(i.name, i.message) for i in health["h"].items if i.level in (1, 2)]}')
        time.sleep(0.5)
    client = ActionClient(node, Harvest, '/atole/mission/harvest')
    client.wait_for_server(timeout_sec=10)
    goal = Harvest.Goal(mode=Harvest.Goal.MODE_ALL if '--all' in args else Harvest.Goal.MODE_NEXT,
                        step_mode='--step' in args, skip_gripper='--gripper' not in args,
                        eih_strategy=opt('--eih', 'none'), eih_dry_run='--dry' in args)
    t0 = time.monotonic()
    gf = client.send_goal_async(goal)
    while not gf.done():
        time.sleep(0.1)
    rf = gf.result().get_result_async()
    while not rf.done():
        time.sleep(0.2)
    res = rf.result().result
    print(f'resultado: ok={res.ok} · {res.message} · cosechados {res.pods_harvested} · fallidos {res.pods_failed} '
          f'· {time.monotonic() - t0:.0f} s')
    print('estados:', ' → '.join(seen))
    for c in checks:
        print('  ', c)
    for e in eih_log:
        print('   EiH:', e)
    time.sleep(0.5)                                  # que llegue el último ArmState
    st = data['status']
    print(f'estado final: {st.state} · {st.message}')
    pos, _ = tcp()
    print('TCP final:', np.round(pos, 4).tolist(), '· dueño del control:', repr(data['arm'].control_owner))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
