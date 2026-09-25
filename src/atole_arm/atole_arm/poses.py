"""Conversiones de pose (ROS ↔ AUBO) y selección de soluciones de IK."""
import math

from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

# Una solución "de frente" tiene J1 a unos 15° del azimut del objetivo (desfase del hombro);
# las "de espaldas" quedan a 165-180°. Medido en la VM del iS10 (2026-09-23).
FACING_MAX_DEG = 90.0
# Objetivos casi sobre la base no tienen azimut fiable.
FACING_MIN_RADIUS_M = 0.10


def quat_to_rpy(qx, qy, qz, qw):
    """Cuaternión → [roll, pitch, yaw] intrínseco ZYX (convención del AUBO)."""
    yaw, pitch, roll = Rotation.from_quat([qx, qy, qz, qw]).as_euler('ZYX')
    return [roll, pitch, yaw]


def rpy_to_quat(roll, pitch, yaw):
    return list(Rotation.from_euler('ZYX', [yaw, pitch, roll]).as_quat())   # x y z w


def pose_to_aubo(pose):
    """geometry_msgs/Pose → [x, y, z, roll, pitch, yaw]."""
    o = pose.orientation
    return [pose.position.x, pose.position.y, pose.position.z] + quat_to_rpy(o.x, o.y, o.z, o.w)


def aubo_to_pose(values):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in values[:3])
    x, y, z, w = rpy_to_quat(*values[3:6])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = x, y, z, w
    return pose


def rotate_about_tool_z(pose_rpy, angle):
    """Gira la orientación de la pose `angle` rad alrededor de su propio eje Z (el de la brida)."""
    r = Rotation.from_euler('ZYX', [pose_rpy[5], pose_rpy[4], pose_rpy[3]]) * Rotation.from_euler('z', angle)
    yaw, pitch, roll = r.as_euler('ZYX')
    return list(pose_rpy[:3]) + [roll, pitch, yaw]


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def facing_offset_deg(joints, pose_rpy):
    """Ángulo entre J1 y el azimut del objetivo. None si el objetivo está casi sobre la base."""
    x, y = pose_rpy[0], pose_rpy[1]
    if math.hypot(x, y) < FACING_MIN_RADIUS_M:
        return None
    return abs(math.degrees(wrap(joints[0] - math.atan2(y, x))))


def pick_facing(seeded, all_solutions, seed, pose_rpy):
    """Devuelve la solución de IK a usar.

    La IK con semilla da la solución más cercana a la postura actual. Desde algunas posturas
    esa solución tiene la base girada ~165° y el brazo llega por detrás. En ese caso se elige,
    entre las soluciones que miran al objetivo, la más cercana a la semilla.
    Devuelve (solución, motivo).
    """
    off = facing_offset_deg(seeded, pose_rpy)
    if off is None or off <= FACING_MAX_DEG:
        return seeded, 'solución con semilla'
    facing = [s for s in all_solutions
              if (facing_offset_deg(s, pose_rpy) or 0.0) <= FACING_MAX_DEG]
    if not facing:
        return seeded, f'ninguna solución mira al objetivo; se usa la de semilla (J1 a {off:.0f}°)'
    best = min(facing, key=lambda s: sum((a - b) ** 2 for a, b in zip(s, seed)))
    return best, (f'la solución con semilla quedaba de espaldas (J1 a {off:.0f}°); '
                  f'se usa una de frente (J1 a {facing_offset_deg(best, pose_rpy):.0f}°)')
