"""config_manager — único dueño de Config.xml.

Lee Config.xml al arrancar, publica su contenido aplanado en /atole/config (latched) y
atiende los cambios. Guarda de forma atómica (fichero temporal + rename) con copia .bak.
Las claves son rutas XML de hojas, p. ej. Robot/Aubo/IP o Poses/Home2/J1.
"""
import os
import shutil
import tempfile
import threading
import xml.etree.ElementTree as ET
from pathlib import Path

from atole_interfaces.msg import ConfigSnapshot
from atole_interfaces.srv import ConfigGet, ConfigSet, SaveCalibration, SavePose
from diagnostic_msgs.msg import KeyValue
from rclpy.node import Node

from atole_common.config_view import CONFIG_TOPIC
from atole_common.qos import LATCHED
from atole_common.stubs import run_node

DEFAULT_PATH = str(Path.home() / 'Documents/pre-Alpha/AtoleROS/config/Config.xml')
POSES = {'home': 'Home', 'home2': 'Home2', 'release': 'Release'}


class ConfigManager(Node):

    def __init__(self):
        super().__init__('config_manager')
        self.path = Path(self.declare_parameter('config_path', DEFAULT_PATH).value).expanduser()
        self._lock = threading.Lock()
        self._version = 0
        self._tree = self._load()
        self._pub = self.create_publisher(ConfigSnapshot, CONFIG_TOPIC, LATCHED)
        self.create_service(ConfigGet, '/atole/config/get', self._on_get)
        self.create_service(ConfigSet, '/atole/config/set', self._on_set)
        self.create_service(SavePose, '/atole/config/save_pose', self._on_save_pose)
        self.create_service(SaveCalibration, '/atole/config/save_calibration', self._on_save_calibration)
        self._publish()
        self.get_logger().info(f'Config.xml cargado: {self.path} ({len(self._flat())} claves)')

    # ── lectura / escritura ──
    def _load(self):
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        return ET.parse(self.path, parser=parser)

    def _save(self):
        """Escritura atómica con copia de seguridad del fichero anterior."""
        if self.path.exists():
            shutil.copy2(self.path, self.path.with_suffix('.xml.bak'))
        fd, tmp = tempfile.mkstemp(dir=self.path.parent, prefix='.Config.', suffix='.xml')
        os.close(fd)
        self._tree.write(tmp, encoding='utf-8', xml_declaration=True)
        os.replace(tmp, self.path)

    def _flat(self):
        out = {}

        def walk(elem, prefix):
            children = [c for c in elem if isinstance(c.tag, str)]
            if not children:
                out[prefix] = (elem.text or '').strip()
            for child in children:
                walk(child, f'{prefix}/{child.tag}' if prefix else child.tag)
        walk(self._tree.getroot(), '')
        out.pop('', None)
        return out

    def _leaf(self, key):
        elem = self._tree.getroot().find(key)
        if elem is None or any(isinstance(c.tag, str) for c in elem):
            return None
        return elem

    def _publish(self):
        self._version += 1
        msg = ConfigSnapshot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.version = self._version
        msg.path = str(self.path)
        msg.entries = [KeyValue(key=k, value=v) for k, v in sorted(self._flat().items())]
        self._pub.publish(msg)

    # ── servicios ──
    def _on_get(self, req, res):
        with self._lock:
            elem = self._leaf(req.key)
            res.found = elem is not None
            res.value = (elem.text or '').strip() if elem is not None else ''
        return res

    def _on_set(self, req, res):
        with self._lock:
            elem = self._leaf(req.key)
            if elem is None:
                res.ok, res.message = False, f'clave inexistente o no es una hoja: {req.key}'
                return res
            elem.text = req.value
            if req.persist:
                self._save()
            self._publish()
        res.ok, res.message = True, f'{req.key} = {req.value}' + (' (guardado)' if req.persist else '')
        self.get_logger().info(res.message)
        return res

    def _on_save_pose(self, req, res):
        tag = POSES.get(req.name.strip().lower())
        if tag is None:
            res.ok, res.message = False, f'pose desconocida: {req.name} (home | home2 | release)'
            return res
        if req.from_current:
            res.ok, res.message = False, 'from_current requiere arm_driver (Fase 1); envía joints_deg'
            return res
        with self._lock:
            for i, value in enumerate(req.joints_deg, start=1):
                self._leaf(f'Poses/{tag}/J{i}').text = f'{value:.4f}'
            self._save()
            self._publish()
        res.ok, res.joints_deg = True, list(req.joints_deg)
        res.message = f'{tag} guardada: {[round(v, 2) for v in req.joints_deg]}'
        self.get_logger().info(res.message)
        return res

    def _on_save_calibration(self, req, res):
        r = req.result
        base = f'Calibrations/{r.camera}'
        with self._lock:
            if self._tree.getroot().find(base) is None:
                res.ok, res.message = False, f'cámara desconocida: {r.camera}'
                return res
            if len(r.matrix) != 16:
                res.ok, res.message = False, 'la matriz debe tener 16 valores'
                return res
            fields = {'Backend': r.backend, 'Method': r.method, 'Notes': r.notes,
                      'Matrix': ' '.join(f'{v:.9g}' for v in r.matrix)}
            for field, value in fields.items():
                leaf = self._leaf(f'{base}/{field}')
                if leaf is not None:
                    leaf.text = value
            self._save()
            self._publish()
        res.ok, res.message = True, f'calibración de {r.camera} guardada ({r.backend}/{r.method})'
        self.get_logger().info(res.message)
        return res


def main(args=None):
    run_node(ConfigManager, args)


if __name__ == '__main__':
    main()
