"""Ayudas para los launch de AtoleROS."""
import os

from launch.actions import ExecuteProcess

DEFAULT_VENV = os.path.expanduser('~/venvs/main312')


def venv_node(package, executable, venv, params=None, name=None):
    """Lanza un nodo con el Python del venv (torch, numpy 2, pyaubo...).

    Mismo patrón que theobroma: `python -m paquete.modulo --ros-args ...`.
    """
    cmd = [os.path.join(venv, 'bin', 'python'), '-m', f'{package}.{executable}',
           '--ros-args', '-r', f'__node:={name or executable}']
    for key, value in (params or {}).items():
        cmd += ['-p', f'{key}:={value}']
    return ExecuteProcess(cmd=cmd, name=name or executable, output='screen')
