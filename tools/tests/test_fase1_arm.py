"""Pruebas de integración de la Fase 1 (arm_driver) contra la VM del AUBO.

Uso (con el sistema lanzado con robot:=vm y el entorno DDS del README exportado):
    ~/venvs/main312/bin/python tools/tests/test_fase1_arm.py
Mueve el robot de la VM y lo deja en Home2. Restaura la pose Home que modifica.
"""
import math
import sys
import threading
import time

import rclpy
from atole_interfaces.action import MoveJoints, MovePose
from atole_interfaces.msg import ArmState
from atole_interfaces.srv import AcquireControl, ConfigGet, JogJoint, SavePose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from atole_common.qos import LATCHED

R = []


def check(name, ok, detail=''):
    R.append((name, ok))
    print(f'  [{"OK" if ok else "FALLO"}] {name}  {detail}', flush=True)


class T(Node):
    def __init__(self):
        super().__init__('test_fase1')
        self.state, self.js_times = None, []
        self.create_subscription(ArmState, '/atole/arm/state', lambda m: setattr(self, 'state', m), LATCHED)
        self.create_subscription(JointState, '/joint_states', lambda m: self.js_times.append(time.monotonic()), 50)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.mj = ActionClient(self, MoveJoints, '/atole/arm/move_joints')
        self.mp = ActionClient(self, MovePose, '/atole/arm/move_pose')

    def srv(self, t, name, req, timeout=10.0):
        c = self.create_client(t, name)
        c.wait_for_service(timeout_sec=5.0)
        return self.wait(c.call_async(req), timeout)

    @staticmethod
    def wait(fut, timeout=60.0):
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def send(self, client, goal, cancel_after=None):
        client.wait_for_server(timeout_sec=5.0)
        gh = self.wait(client.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            return None, 'rechazado', None
        t_cancel = None
        if cancel_after is not None:
            time.sleep(cancel_after)
            t_cancel = time.monotonic()
            self.wait(gh.cancel_goal_async(), 5.0)
        res = self.wait(gh.get_result_async(), 120.0)
        return res.result if res else None, res.status if res else None, t_cancel


def main():
    rclpy.init()
    n = T()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    time.sleep(3.0)

    print('1) estado', flush=True)
    t0 = time.monotonic(); n.js_times.clear(); time.sleep(3.0)
    rate = len([t for t in n.js_times if t >= t0]) / 3.0
    check('/joint_states >= 18 Hz', rate >= 18, f'{rate:.1f} Hz')
    s = n.state
    check('robot conectado y Running', s is not None and s.connected and s.robot_mode == 'Running',
          f'{s.robot_mode if s else None} / {s.safety_mode if s else None}')

    print('2) URDF + TF del TCP frente al controlador', flush=True)
    try:
        tr = n.tf.lookup_transform('base_link', 'tcp', rclpy.time.Time())
        p = tr.transform.translation
        tcp = n.state.tcp_pose.position
        err = math.dist((p.x, p.y, p.z), (tcp.x, tcp.y, tcp.z)) * 1000
        check('TF base_link→tcp (URDF) = TCP del controlador', err < 1.0, f'{err:.3f} mm')
    except Exception as e:
        check('TF base_link→tcp disponible', False, str(e))

    print('3) lease', flush=True)
    r = n.srv(AcquireControl, '/atole/arm/acquire_control', AcquireControl.Request(client_id='test'))
    check('acquire_control("test")', r and r.ok, r.message if r else '')
    r = n.srv(JogJoint, '/atole/arm/jog_joint', JogJoint.Request(client_id='gui', joint_index=0, delta_deg=5.0))
    check('jog de "gui" rechazado con lease ajeno', r and not r.ok, r.message if r else '')

    home2 = [math.radians(float(n.srv(ConfigGet, '/atole/config/get', ConfigGet.Request(key=f'Poses/Home2/J{i}')).value))
             for i in range(1, 7)]

    def move_j(q, speed=1.0, bypass=True, cancel_after=None):
        return n.send(n.mj, MoveJoints.Goal(client_id='test', joints=q, speed=speed, accel=1.0, bypass_j6_lock=bypass),
                      cancel_after=cancel_after)

    print('4) move_joints a Home2', flush=True)
    t = time.monotonic(); res, st, _ = move_j(home2)
    err = max(abs(a - b) for a, b in zip(n.state.joints, home2))
    check('llega a Home2', res and res.ok and math.degrees(err) < 0.5,
          f'{res.message if res else st} · error {math.degrees(err):.2f}° · {time.monotonic() - t:.1f} s')

    print('5) cancelación a mitad de movimiento', flush=True)
    far = list(home2); far[0] += math.radians(60)
    res, st, t_cancel = move_j(far, speed=0.3, cancel_after=1.0)
    while n.state.is_moving and time.monotonic() - t_cancel < 5:
        time.sleep(0.01)
    check('cancel → CANCELED y robot quieto', st == 5 and not n.state.is_moving,
          f'estado {st} · quieto {1000 * (time.monotonic() - t_cancel):.0f} ms tras cancelar · J1 {math.degrees(n.state.joints[0] - home2[0]):.1f}° recorridos')

    print('6) move_pose J_IK desde la postura que daba "de espaldas" (caso 20:26)', flush=True)
    res, _, _ = move_j([0.0, 0.5236, -2.7715, -1.3961, 1.5708, 0.0])
    check('a la postura inicial de la VM', res and res.ok, res.message if res else '')
    pose = PoseStamped(); pose.header.frame_id = 'base_link'
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 0.9599, -0.0897, 0.0663
    q = Rotation.from_rotvec([0.4409, 0.3992, 0.2169]).as_quat()
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
    res, st, _ = n.send(n.mp, MovePose.Goal(client_id='test', target=pose, move_type=1, speed=1.0, accel=1.0, prefer_facing=True))
    j1 = math.degrees(res.ik_solution[0]) if res else float('nan')
    az = math.degrees(math.atan2(-0.0897, 0.9599))
    check('IK elige solución de frente', res and res.ok and abs(j1 - az) < 90, f'J1 = {j1:.1f}° (azimut {az:.1f}°) · {res.message if res else st}')

    print('7) moveL con J6 bloqueado', flush=True)
    n.srv(Trigger, '/atole/arm/stop', Trigger.Request())
    j6_before = n.state.joints[5]
    tgt = PoseStamped(); tgt.header.frame_id = 'base_link'; tgt.pose = n.state.tcp_pose
    tgt.pose.position.z += 0.05
    rq = Rotation.from_quat([tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w]) \
        * Rotation.from_euler('z', math.radians(25))
    tgt.pose.orientation.x, tgt.pose.orientation.y, tgt.pose.orientation.z, tgt.pose.orientation.w = rq.as_quat()
    goal_z = tgt.pose.position.z
    res, st, _ = n.send(n.mp, MovePose.Goal(client_id='test', target=tgt, move_type=0, speed=0.1, accel=0.3))
    dj6 = math.degrees(abs(n.state.joints[5] - j6_before))
    dz = abs(n.state.tcp_pose.position.z - goal_z) * 1000
    check('moveL: sube 5 cm, llega al mm y J6 no se mueve (pedía girar 25°)', res and res.ok and dj6 < 0.5 and dz < 1.0,
          f'{res.message if res else st} · ΔJ6 {dj6:.2f}° · error z {dz:.2f} mm')

    print('8) stop a mitad de movimiento', flush=True)
    th = threading.Thread(target=lambda: setattr(n, '_r8', move_j(far, speed=0.3)))
    th.start(); time.sleep(1.2)
    t_stop = time.monotonic(); r = n.srv(Trigger, '/atole/arm/stop', Trigger.Request())
    th.join(timeout=10)
    res8 = n._r8[0]
    check('stop detiene el movimiento y la acción informa', r and r.success and res8 and not res8.ok,
          f'acción: {res8.message if res8 else None} · {1000 * (time.monotonic() - t_stop):.0f} ms')

    print('9) save_pose desde la posición actual (y restaurar)', flush=True)
    orig = [float(n.srv(ConfigGet, '/atole/config/get', ConfigGet.Request(key=f'Poses/Home/J{i}')).value) for i in range(1, 7)]
    r = n.srv(SavePose, '/atole/config/save_pose', SavePose.Request(name='home', from_current=True))
    cur = [math.degrees(v) for v in n.state.joints]
    check('save_pose(from_current) guarda las articulaciones actuales', r and r.ok and max(abs(a - b) for a, b in zip(r.joints_deg, cur)) < 0.5,
          r.message if r else '')
    r = n.srv(SavePose, '/atole/config/save_pose', SavePose.Request(name='home', from_current=False, joints_deg=orig))
    check('Home original restaurada', r and r.ok, r.message if r else '')

    print('10) vuelta a Home2 y liberar lease', flush=True)
    res, _, _ = move_j(home2)
    check('de vuelta en Home2', res and res.ok, res.message if res else '')
    r = n.srv(AcquireControl, '/atole/arm/acquire_control', AcquireControl.Request(client_id='test', release=True))
    check('lease liberado', r and r.ok and r.owner == '', r.message if r else '')

    ok = sum(1 for _, o in R if o)
    print(f'\nRESULTADO: {ok}/{len(R)} pruebas OK', flush=True)
    rclpy.shutdown()
    sys.exit(0 if ok == len(R) else 1)


if __name__ == '__main__':
    main()
