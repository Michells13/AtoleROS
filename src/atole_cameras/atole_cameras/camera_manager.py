"""camera_manager — mantiene activas la cámara EtH seleccionada (cam0 | cam1 | camC) y cam2.

Cada cámara se lanza con el launch oficial de zed-ros2-wrapper en su propio grupo de procesos:
    namespace atole/zed, camera_name <id>  →  topics /atole/zed/<id>/...
publish_urdf:=true (la cadena <id>_camera_link → … → óptico la necesita la estabilización de
profundidad y es donde cuelga la calibración), publish_tf:=false, sin positional tracking propio.

- Detecta las cámaras conectadas con pyzed (serial → id según Config.xml) al arrancar, al cambiar de EtH
  y cuando falta una cámara que debería estar activa (que entonces se rearranca).
- "streaming" = llega su /status/heartbeat.
- Cambiar de EtH: SIGINT al grupo de la cámara anterior (la ZED tarda ~15 s en cerrarse),
  SIGKILL si no responde en 25 s, y luego se lanza la nueva. La selección se guarda en Config.xml.
- Con sim:=true no lanza cámaras (las publica dataset_player).
"""
import json
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from atole_interfaces.action import SetCameraMode
from atole_interfaces.msg import CameraSlot, CameraStatus
from atole_interfaces.srv import ConfigSet, SelectEth, SetDisplay
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from zed_msgs.msg import Heartbeat

from atole_common.config_view import ConfigView
from atole_common.qos import LATCHED
from atole_common.stubs import run_node

CAMERAS = ('cam0', 'cam1', 'camC', 'cam2')
ETH = ('cam0', 'cam1', 'camC')
STOP_TIMEOUT_S = 25.0
HEARTBEAT_TIMEOUT_S = 3.0
LOG_DIR = Path.home() / '.ros' / 'log'


LIST_DEVICES = ('import json, pyzed.sl as sl; '
                'print(json.dumps({str(d.serial_number): str(d.camera_model) for d in sl.Camera.get_device_list()}))')


def list_zed_devices():
    """{serial: modelo} con pyzed en un subproceso: así la conexión con nvargus-daemon se cierra al
    terminar (en este proceso quedaría abierta para siempre) y un fallo de pyzed no tumba el nodo."""
    out = subprocess.run(['python3', '-c', LIST_DEVICES], capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        raise RuntimeError(out.stderr.strip().splitlines()[-1] if out.stderr.strip() else f'código {out.returncode}')
    return json.loads(out.stdout.strip().splitlines()[-1])


def container_name(cam):
    return f'zed_container_{cam}'


def foreign_containers(cam):
    """PIDs de contenedores ZED de esta cámara que ya están corriendo (de cualquier proceso)."""
    out = subprocess.run(['pgrep', '-f', f'__node:={container_name(cam)}( |$)'], capture_output=True, text=True).stdout
    return out.split()


class CameraManager(Node):

    def __init__(self):
        super().__init__('camera_manager')
        self.sim = self.declare_parameter('sim', False).value
        self.zed_setup = os.path.expanduser(self.declare_parameter(
            'zed_setup', '~/ros2_ws/install/setup.bash').value)
        self.procs = {}              # id -> subprocess.Popen
        self.heartbeats = {}         # id -> monotonic del último heartbeat
        self.connected = {}          # serial -> modelo (pyzed)
        self.swapping = False
        self._lock = threading.RLock()
        self._started = False
        self._startup_done = False
        self._eth = None             # EtH elegida en este proceso (Config.xml se actualiza de forma asíncrona)
        group = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(CameraStatus, '/atole/cameras/status', LATCHED)
        self.config_set = self.create_client(ConfigSet, '/atole/config/set', callback_group=group)
        self.create_service(SelectEth, '/atole/cameras/select_eth', self._srv_select, callback_group=group)
        self.create_service(SetDisplay, '/atole/cameras/set_display', self._srv_display, callback_group=group)
        ActionServer(self, SetCameraMode, '/atole/cameras/set_mode', execute_callback=self._exec_mode,
                     goal_callback=lambda _: GoalResponse.REJECT if self.swapping else GoalResponse.ACCEPT,
                     callback_group=group)
        for cam in CAMERAS:
            self.create_subscription(Heartbeat, f'/atole/zed/{cam}/status/heartbeat',
                                     lambda _m, cam=cam: self.heartbeats.__setitem__(cam, time.monotonic()), 10)
        self.config = ConfigView(self, on_update=self._on_config)
        self.create_timer(1.0, self._publish_status, callback_group=group)
        self.create_timer(10.0, self._detect_if_missing, callback_group=group)
        self.get_logger().info(f'camera_manager arrancado (fuente: {"sim" if self.sim else "cámaras reales"})')

    # ───────────────────────── configuración y arranque ─────────────────────────
    def _cam(self, cam, field, default=''):
        return self.config.get(f'Cameras/{cam}/{field}', default)

    def _on_config(self, _cfg):
        if self._started:
            return
        self._started = True
        if self.sim:
            return
        threading.Thread(target=self._startup, daemon=True).start()

    def _startup(self):
        self._detect()
        eth = self.config.get('Cameras/SelectedEtH', '')
        for cam in (eth, 'cam2'):
            error = self._can_start(cam)
            if error:
                self.get_logger().error(f'no se arranca {cam}: {error} (se reintenta al detectarla)')
                continue
            self._start(cam)
            # Escalonado: abrir dos ZED a la vez hace que cada SDK sondee el sensor que el otro está
            # abriendo (Argus "Device in use"); se espera a que la primera transmita.
            if cam != 'cam2' and not self._wait_stream(cam, timeout=60.0):
                self.get_logger().warn(f'{cam} no transmite tras 60 s; se sigue con la siguiente cámara')
        self._startup_done = True

    def _required(self):
        return [c for c in (self._eth or self.config.get('Cameras/SelectedEtH', ''), 'cam2') if c]

    def _detect_if_missing(self):
        """Solo consulta pyzed si falta alguna cámara que debería estar activa: cada consulta abre una
        conexión con nvargus-daemon, y no conviene hacerlo en bucle mientras las cámaras transmiten."""
        if self._startup_done and not self.swapping and any(
                not (c in self.procs and self.procs[c].poll() is None) for c in self._required()):
            self._detect()

    def _detect(self):
        if self.sim:
            return
        try:
            self.connected = list_zed_devices()
        except Exception as e:
            self.get_logger().warn(f'no se pudo listar las cámaras con pyzed: {e}', throttle_duration_sec=60.0)
            return
        self._restart_missing()
        known = {self._cam(c, 'Serial') for c in CAMERAS}
        for serial, model in self.connected.items():
            if serial not in known:
                self.get_logger().warn(f'cámara conectada sin asignar en Config.xml: {model} serial {serial}',
                                       throttle_duration_sec=300.0)

    def _restart_missing(self):
        """Arranca la EtH seleccionada o cam2 si deberían estar activas y no lo están: una cámara que no
        estaba al arrancar (p. ej. la ZED anterior aún se cerraba) o cuyo proceso terminó."""
        if not self._startup_done or self.swapping:
            return
        for cam in self._required():
            running = cam in self.procs and self.procs[cam].poll() is None
            if not running and self._can_start(cam) is None:
                self.get_logger().warn(f'{cam} debería estar activa y no lo está: se arranca')
                self._start(cam)

    def _can_start(self, cam):
        if cam not in CAMERAS:
            return f'cámara desconocida "{cam}"'
        serial = self._cam(cam, 'Serial')
        if not serial:
            return f'{cam} no tiene serial en Config.xml (Cameras/{cam}/Serial)'
        if self.connected and serial not in self.connected:
            return f'{cam} (serial {serial}) no está conectada'
        return None

    # ───────────────────────── procesos ZED ─────────────────────────
    def _start(self, cam):
        with self._lock:
            if cam in self.procs and self.procs[cam].poll() is None:
                return
            others = foreign_containers(cam)
            if others:
                self.get_logger().error(f'no se lanza {cam}: ya hay un contenedor ZED suyo (pid {", ".join(others)}); '
                                        '¿otra instancia de AtoleROS o un proceso huérfano? Ciérralo antes')
                return
            depth = self._cam(cam, 'DepthMode', 'NEURAL_PLUS')
            overrides = ';'.join([f'depth.depth_mode:={depth}', 'depth.publish_depth_confidence:=true',
                                  'pos_tracking.pos_tracking_enabled:=false', 'general.pub_resolution:=NATIVE'])
            # Launch propio: un contenedor con nombre propio por cámara (ver launch/zed_camera.launch.py).
            launch = (f'ros2 launch atole_cameras zed_camera.launch.py camera_model:={self._cam(cam, "Model")} '
                      f'camera_name:={cam} namespace:=atole/zed serial_number:={self._cam(cam, "Serial")} '
                      f'publish_urdf:=true publish_tf:=false publish_map_tf:=false "param_overrides:={overrides}"')
            LOG_DIR.mkdir(parents=True, exist_ok=True)
            log = open(LOG_DIR / f'atole_zed_{cam}.log', 'w')
            self.procs[cam] = subprocess.Popen(['bash', '-c', f'source {self.zed_setup} && exec {launch}'],
                                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            self.heartbeats.pop(cam, None)
            self.get_logger().info(f'{cam} lanzada (serial {self._cam(cam, "Serial")}, {depth}); '
                                   f'log en {LOG_DIR}/atole_zed_{cam}.log')

    def _stop(self, cam):
        with self._lock:
            proc = self.procs.pop(cam, None)
        if proc is None or proc.poll() is not None:
            return True
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            proc.wait(timeout=STOP_TIMEOUT_S)
            clean = True
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f'{cam} no se cerró en {STOP_TIMEOUT_S:.0f} s: SIGKILL')
            os.killpg(pgid, signal.SIGKILL)
            proc.wait(timeout=5)
            subprocess.run(['bash', '-c', f'source {self.zed_setup} && fastdds shm clean'],
                           capture_output=True, timeout=20)
            clean = False
        try:                      # por si quedó algún hijo del grupo (el contenedor de la ZED)
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.heartbeats.pop(cam, None)
        self.get_logger().info(f'{cam} detenida')
        return clean

    def _streaming(self, cam):
        t = self.heartbeats.get(cam)
        return t is not None and time.monotonic() - t < HEARTBEAT_TIMEOUT_S

    def _wait_stream(self, cam, timeout=120.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._streaming(cam):
                return True
            if cam in self.procs and self.procs[cam].poll() is not None:
                return False
            time.sleep(0.5)
        return False

    # ───────────────────────── cambio de EtH ─────────────────────────
    def _swap(self, new, feedback=None):
        if self.sim:
            return False, 'en modo SIM no se lanzan cámaras'
        if new not in ETH:
            return False, f'{new} no es una cámara EtH (cam0 | cam1 | camC)'
        self._detect()
        error = self._can_start(new)
        if error:
            return False, error
        self.swapping = True
        try:
            for cam in ETH:
                if cam != new and cam in self.procs:
                    feedback and feedback('unload')
                    self._stop(cam)
            feedback and feedback('load')
            self._eth = new
            self._start(new)
            if self.config_set.service_is_ready():
                self.config_set.call_async(ConfigSet.Request(key='Cameras/SelectedEtH', value=new, persist=True))
            feedback and feedback('wait_stream')
            if not self._wait_stream(new):
                return False, f'{new} no empezó a transmitir (revisa {LOG_DIR}/atole_zed_{new}.log)'
            return True, f'{new} activa y transmitiendo'
        finally:
            self.swapping = False

    def _srv_select(self, req, res):
        res.ok, res.message = self._swap(req.camera)
        return res

    def _exec_mode(self, gh):
        g, result = gh.request, SetCameraMode.Result()
        if g.source and g.source != ('sim' if self.sim else 'live'):
            gh.abort()
            result.ok, result.message = False, 'cambiar entre SIM y cámaras reales requiere relanzar (sim:=true|false)'
            return result
        ok, msg = self._swap(g.active_eth, lambda phase: gh.publish_feedback(SetCameraMode.Feedback(phase=phase)))
        (gh.succeed if ok else gh.abort)()
        result.ok, result.message, result.status = ok, msg, self._publish_status()
        return result

    def _srv_display(self, req, res):
        what = req.what or 'image'
        if req.camera not in CAMERAS or what not in ('image', 'cloud'):
            res.ok, res.message = False, f'cámara "{req.camera}" o tipo "{what}" desconocidos (image | cloud)'
            return res
        key = f'Cameras/{req.camera}/{"Display" if what == "image" else "Cloud"}'
        if self.config_set.service_is_ready():
            self.config_set.call_async(ConfigSet.Request(key=key, value=str(bool(req.enable)).lower(), persist=True))
        res.ok = True
        res.message = f'{"imagen" if what == "image" else "nube"} de {req.camera} {"activada" if req.enable else "desactivada"}'
        return res

    # ───────────────────────── estado ─────────────────────────
    def _publish_status(self):
        msg = CameraStatus(source='sim' if self.sim else 'live', active_eth=self.config.get('Cameras/SelectedEtH', ''),
                           swap_in_progress=self.swapping)
        msg.header.stamp = self.get_clock().now().to_msg()
        for cam in CAMERAS:
            serial = self._cam(cam, 'Serial')
            calib = self.config.section(f'Calibrations/{cam}')
            active = cam in self.procs and self.procs[cam].poll() is None
            msg.cameras.append(CameraSlot(
                id=cam, model=self._cam(cam, 'Model'), serial=serial, role=self._cam(cam, 'Role'),
                position=self._cam(cam, 'Position'), connected=bool(serial) and serial in self.connected,
                active=active or (self.sim and self._streaming(cam)), streaming=self._streaming(cam),
                calibrated=bool(calib.get('Matrix')) and bool(serial) and calib.get('Serial') == serial,
                display=self._cam(cam, 'Display', 'true') == 'true',
                cloud=self._cam(cam, 'Cloud', 'false') == 'true'))
        self.status_pub.publish(msg)
        return msg

    def destroy_node(self):
        # En paralelo: cada ZED tarda ~15 s en cerrarse. El launch da tiempo con sigterm_timeout.
        threads = [threading.Thread(target=self._stop, args=(cam,)) for cam in list(self.procs)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        super().destroy_node()


def main(args=None):
    run_node(CameraManager, args)


if __name__ == '__main__':
    main()
