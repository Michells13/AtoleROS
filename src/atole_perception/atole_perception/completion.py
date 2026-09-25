"""Completion con AdaPoinTr en workers calientes (un subproceso por checkpoint).

AdaPoinTr necesita extensiones CUDA compiladas (pointnet2_ops, chamfer) que viven en su propio
venv, así que corre en un subproceso: `<PythonExe> <Worker> --serve --ckpt <ckpt> --device cuda`.
Protocolo (el de PozoleV3): el worker imprime {"ready": true}; por petición recibe una línea JSON
{"in": partial.npy, "out": dense.npy, "seed": N} y responde {"ok": true, ...}. La nube vuelve en el
mismo marco y unidades que la entrada.

El worker remuestrea la parcial a 2048 puntos, normaliza al bbox unidad, predice 16384 puntos y
deshace la normalización. Con el modelo cargado, cada petición tarda ~0.1–0.5 s; arrancar un
worker tarda ~8 s, por eso se precalientan al inicio (warm-up).
"""
import json
import queue
import shutil
import subprocess
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

STARTUP_TIMEOUT_S = 90.0
MAX_WORKERS = 2          # EtH + EiH residentes; más modelos a la vez pueden agotar la memoria del Orin


class Worker:
    """Un subproceso `--serve` con lectores en segundo plano de stdout y stderr."""

    def __init__(self, python_exe, worker, ckpt, device='cuda'):
        self.ckpt = str(ckpt)
        self.tmp = Path(tempfile.mkdtemp(prefix='atole_adapointr_'))
        self.in_npy, self.out_npy = self.tmp / 'partial.npy', self.tmp / 'dense.npy'
        self.proc = subprocess.Popen([str(python_exe), str(worker), '--serve', '--ckpt', self.ckpt, '--device', device],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, bufsize=1)
        self._lines = queue.Queue()
        self._err = deque(maxlen=30)
        threading.Thread(target=self._pump_out, daemon=True).start()
        threading.Thread(target=self._pump_err, daemon=True).start()

    def _pump_out(self):
        for line in self.proc.stdout:
            self._lines.put(line)
        self._lines.put(None)

    def _pump_err(self):
        for line in self.proc.stderr:
            self._err.append(line.rstrip())

    def err_tail(self, n=8):
        return '\n'.join(list(self._err)[-n:])

    def alive(self):
        return self.proc.poll() is None

    def _read_json(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('el worker no respondió a tiempo')
            line = self._lines.get(timeout=remaining)
            if line is None:
                raise RuntimeError('el worker terminó')
            line = line.strip()
            if line.startswith('{'):          # salta el ruido que no es JSON (banners del modelo)
                try:
                    return json.loads(line)
                except json.JSONDecodeError:
                    continue

    def wait_ready(self, timeout=STARTUP_TIMEOUT_S):
        msg = self._read_json(timeout)
        if not msg.get('ready'):
            raise RuntimeError(f'el worker no reportó ready: {msg}')
        return msg

    def infer(self, points, seed, timeout):
        np.save(self.in_npy, np.asarray(points, np.float32))
        self.proc.stdin.write(json.dumps({'in': str(self.in_npy), 'out': str(self.out_npy), 'seed': int(seed)}) + '\n')
        self.proc.stdin.flush()
        meta = self._read_json(timeout)
        if not meta.get('ok'):
            raise RuntimeError(meta.get('error', 'el worker reportó un fallo'))
        return np.load(self.out_npy).astype(np.float32), meta

    def close(self):
        try:
            self.proc.stdin.write(json.dumps({'cmd': 'quit'}) + '\n')
            self.proc.stdin.flush()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()
        shutil.rmtree(self.tmp, ignore_errors=True)


class CompletionPool:
    """Workers por checkpoint (LRU, máximo MAX_WORKERS). Seguro entre hilos: serializa el uso de la GPU."""

    def __init__(self, logger=None):
        self._workers = {}
        self._lock = threading.Lock()
        self._log = logger

    def _get(self, python_exe, worker, ckpt):
        for path, what in ((python_exe, 'PythonExe'), (worker, 'Worker'), (ckpt, 'checkpoint')):
            if not path or not Path(path).exists():
                raise FileNotFoundError(f'no existe {what}: {path}')
        key = (str(python_exe), str(worker), str(ckpt))
        w = self._workers.pop(key, None)
        if w is not None and w.alive():
            self._workers[key] = w                   # al final = usado más recientemente
            return w
        if w is not None:
            w.close()
        t0 = time.monotonic()
        w = Worker(python_exe, worker, ckpt)
        try:
            w.wait_ready()
        except Exception as e:
            tail = w.err_tail()
            w.close()
            raise RuntimeError(f'el worker de AdaPoinTr no arrancó: {e}' + (f'\n{tail}' if tail else ''))
        if self._log:
            self._log.info(f'worker AdaPoinTr listo en {time.monotonic() - t0:.1f} s ({Path(ckpt).name})')
        self._workers[key] = w
        while len(self._workers) > MAX_WORKERS:
            old = next(iter(self._workers))
            self._workers.pop(old).close()
        return w

    def complete(self, points, python_exe, worker, ckpt, seed=0, timeout=120.0):
        """Devuelve (nube densa (M,3) float32, meta). Un reintento si el worker murió entre llamadas."""
        key = (str(python_exe), str(worker), str(ckpt))
        with self._lock:
            last = None
            for _attempt in (1, 2):
                try:
                    return self._get(python_exe, worker, ckpt).infer(points, seed, timeout)
                except (TimeoutError, queue.Empty):
                    self._drop(key)
                    raise TimeoutError(f'completion sin respuesta en {timeout:.0f} s')
                except FileNotFoundError:
                    raise
                except Exception as e:
                    w = self._workers.get(key)
                    last = f'{type(e).__name__}: {e}' + (f'\n{w.err_tail()}' if w else '')
                    self._drop(key)
            raise RuntimeError(f'completion falló: {last}')

    def _drop(self, key):
        w = self._workers.pop(key, None)
        if w is not None:
            w.close()

    def shutdown(self):
        with self._lock:
            for w in self._workers.values():
                w.close()
            self._workers.clear()
