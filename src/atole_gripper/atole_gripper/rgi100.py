"""Driver Modbus RTU del gripper DH-Robotics RGI-100 (grip + rotación).

Apertura 0..1000 donde 1000 = abierto (así se comporta el gripper; PozoleV3 abre con 1000).
Los registros de feedback (posición, corriente, ángulo y estado de rotación) se verificaron
leyendo el gripper real el 2026-09-25.
"""
import glob
import threading
import time

import minimalmodbus

# Registros de comando
INIT = 0x0100            # 0x01 inicialización normal, 0xA5 completa
FORCE = 0x0101           # 20..100 %
POSITION = 0x0103        # 0..1000
SPEED = 0x0104           # 1..100 %
ROT_ANGLE = 0x0105       # grados absolutos, con signo
ROT_SPEED = 0x0107       # 1..100 %
ROT_FORCE = 0x0108       # 20..100 %
INIT_DIRECTION = 0x0301  # 0 abre al inicializar, 1 cierra
# Registros de feedback
INIT_STATE = 0x0200      # 0 sin inicializar, 1 listo, 2 inicializando
GRIP_STATE = 0x0201      # 0 moviendo, 1 llegó, 2 agarró objeto, 3 objeto caído
POSITION_FB = 0x0202
CURRENT_FB = 0x0204
ROT_ANGLE_FB = 0x0208
ROT_STATE = 0x020B       # 0 moviendo, 1 llegó, 2 bloqueado, 3 bloqueo liberado

INIT_NAMES = {0: 'SIN_INICIALIZAR', 1: 'LISTO', 2: 'INICIALIZANDO'}
GRIP_NAMES = {0: 'MOVIENDO', 1: 'LLEGÓ', 2: 'AGARRÓ', 3: 'CAÍDO'}


class GripperError(RuntimeError):
    pass


def clamp(value, low, high):
    return max(low, min(high, int(value)))


class Rgi100:

    def __init__(self, port, slave=1, baud=115200, parity='N', stopbits=1, timeout=1.0):
        self.port, self.slave = port, int(slave)
        self.baud, self.parity, self.stopbits, self.timeout = int(baud), parity, int(stopbits), float(timeout)
        self._inst = None
        self._lock = threading.Lock()

    def open(self):
        inst = minimalmodbus.Instrument(self.port, self.slave, mode=minimalmodbus.MODE_RTU)
        inst.serial.baudrate, inst.serial.parity = self.baud, self.parity
        inst.serial.stopbits, inst.serial.timeout = self.stopbits, self.timeout
        inst.clear_buffers_before_each_transaction = True
        self._inst = inst
        self.read_register(INIT_STATE)       # falla aquí si no es un RGI-100

    def close(self):
        if self._inst is not None:
            try:
                self._inst.serial.close()
            except Exception:
                pass
        self._inst = None

    @property
    def connected(self):
        return self._inst is not None

    def read_register(self, reg, signed=False, retries=2):
        for attempt in range(retries + 1):
            try:
                with self._lock:
                    return self._inst.read_register(reg, 0, functioncode=3, signed=signed)
            except Exception as e:
                if attempt == retries:
                    raise GripperError(f'lectura 0x{reg:04X} falló: {e}') from e
                time.sleep(0.05)

    def write_register(self, reg, value, signed=False, retries=2):
        for attempt in range(retries + 1):
            try:
                with self._lock:
                    self._inst.write_register(reg, int(value), functioncode=6, signed=signed)
                return
            except Exception as e:
                if attempt == retries:
                    raise GripperError(f'escritura 0x{reg:04X} falló: {e}') from e
                time.sleep(0.05)

    def read_state(self):
        return {
            'init_state': self.read_register(INIT_STATE),
            'grip_state': self.read_register(GRIP_STATE),
            'position': self.read_register(POSITION_FB),
            'current': self.read_register(CURRENT_FB),
            'angle': self.read_register(ROT_ANGLE_FB, signed=True),
            'rot_state': self.read_register(ROT_STATE),
        }

    def initialize(self, full=True, direction=0, timeout=15.0):
        """Inicializa el gripper. MUEVE el gripper hasta sus topes."""
        self.write_register(INIT_DIRECTION, 1 if direction else 0)
        self.write_register(INIT, 0xA5 if full else 0x01)
        t0 = time.monotonic()
        time.sleep(0.3)
        while time.monotonic() - t0 < timeout:
            if self.read_register(INIT_STATE) == 1:
                return
            time.sleep(0.2)
        raise GripperError(f'el gripper no terminó de inicializarse en {timeout:.0f} s')

    def move(self, opening, force, speed):
        """Envía la apertura (0..1000, 1000 = abierto). No espera."""
        self.write_register(FORCE, clamp(force, 20, 100))
        self.write_register(SPEED, clamp(speed, 1, 100))
        self.write_register(POSITION, clamp(opening, 0, 1000))

    def rotate(self, angle_deg, force, speed):
        """Gira a un ángulo absoluto en grados (con signo). No espera."""
        self.write_register(ROT_SPEED, clamp(speed, 1, 100))
        self.write_register(ROT_FORCE, clamp(force, 20, 100))
        self.write_register(ROT_ANGLE, clamp(angle_deg, -32768, 32767), signed=True)


def scan(candidates=None, **kwargs):
    """Busca el RGI-100 en los puertos serie. Devuelve el puerto o None."""
    ports = candidates or sorted(glob.glob('/dev/serial/by-id/*FTDI*') + glob.glob('/dev/ttyUSB*'))
    for port in ports:
        g = Rgi100(port, **{**kwargs, 'timeout': 0.3})
        try:
            g.open()
            return port
        except Exception:
            continue
        finally:
            g.close()
    return None
