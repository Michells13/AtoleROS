"""Vista de solo lectura de Config.xml, alimentada por /atole/config (config_manager)."""
import threading

from atole_interfaces.msg import ConfigSnapshot

from atole_common.qos import LATCHED

CONFIG_TOPIC = '/atole/config'


class ConfigView:
    """Mantiene la última copia de la configuración publicada por config_manager."""

    def __init__(self, node, on_update=None):
        self._values = {}
        self.version = 0
        self._on_update = on_update
        self._received = threading.Event()
        node.create_subscription(ConfigSnapshot, CONFIG_TOPIC, self._callback, LATCHED)

    def _callback(self, msg):
        self._values = {kv.key: kv.value for kv in msg.entries}
        self.version = msg.version
        self._received.set()
        if self._on_update:
            self._on_update(self)

    @property
    def ready(self):
        return bool(self._values)

    def wait_ready(self, timeout=10.0):
        """Espera a la primera configuración (los servicios pueden llegar antes que el topic latched)."""
        return self._received.wait(timeout)

    def get(self, key, default=None):
        return self._values.get(key, default)

    def get_float(self, key, default=0.0):
        try:
            return float(self._values[key])
        except (KeyError, ValueError):
            return default

    def get_bool(self, key, default=False):
        value = self._values.get(key)
        return default if value is None else value.strip().lower() in ('1', 'true', 'yes', 'si', 'sí')

    def section(self, prefix):
        """Claves bajo un prefijo, sin el prefijo: section('Poses/Home2') -> {'J1': ...}."""
        prefix = prefix.rstrip('/') + '/'
        return {k[len(prefix):]: v for k, v in self._values.items() if k.startswith(prefix)}
