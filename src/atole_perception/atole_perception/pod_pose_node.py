"""pod_pose_node — pose de cada pod: máscara → nube → área de trabajo → completion → superquadric → PCA.

Acción /atole/perception/estimate_pods (EstimatePods):
  1. capture   congela un frame sincronizado de la cámara (RGB, profundidad, confianza y K con el
               mismo stamp; en SIM lo publica dataset_player).
  2. detect    pide a detector_node las máscaras sobre ESE frame (DetectOnce con la imagen).
  3. cloud     máscara → nube parcial (filtros de PozoleV3, mask_cloud.py) y centroide en base_link
               por TF (base_link ← <cam>_calibrated_optical, la calibración exacta de Config.xml).
               Filtro de área de trabajo sobre ese centroide (Perception/Workspace). En EiH, si hay
               referencia, solo sigue la detección más cercana dentro de gate_m.
  4. completion, superquadric y PCA en el marco de la cámara (mismo código que PozoleV3), y el
               resultado se pasa a base_link: puntos con T, vectores con R, orientación R·q.
Publica /atole/perception/pods, las nubes parcial y completada (PointCloud2 en base_link, campo
`id` = pod) y marcadores. /atole/perception/pod_pose/warmup arranca los workers EtH y EiH.
"""
import threading
import time

import numpy as np
from atole_interfaces.action import EstimatePods
from atole_interfaces.msg import Pod, PodArray
from atole_interfaces.srv import DetectOnce
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from atole_common.config_view import ConfigView
from atole_common.stubs import run_node
from atole_perception import mask_cloud
from atole_perception.completion import CompletionPool
from atole_perception.pca_pose import estimate_pose
from atole_perception.preprocess import preprocess_for_completion
from atole_perception.ros_utils import (cloud_msg, cloud_to_array, image_to_array, matrix_to_pose, point,
                                         pose_to_matrix, quaternion, transform_to_matrix, vector)
from atole_perception.superquadric import fit_and_sample

BASE = 'base_link'
CAPTURE_TIMEOUT_S = 8.0
DETECT_TIMEOUT_S = 60.0          # incluye cargar el modelo si no hubo warm-up
SENSOR_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)    # ver IMAGE_QOS en detector_node
SQ_EIH = {'max_pts': 800, 'max_iter': 100}      # PozoleV3 aligera el superquadric en EiH
EIH_MIN_POINTS = 120


class PodPoseNode(Node):

    def __init__(self):
        super().__init__('pod_pose_node')
        self.group = ReentrantCallbackGroup()
        self.config = ConfigView(self)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.pool = CompletionPool(self.get_logger())
        self.busy = False
        self.detect_client = self.create_client(DetectOnce, '/atole/perception/detect_once', callback_group=self.group)
        self.pods_pub = self.create_publisher(PodArray, '/atole/perception/pods', 10)
        self.partial_pub = self.create_publisher(PointCloud2, '/atole/perception/cloud/partial', 2)
        self.completed_pub = self.create_publisher(PointCloud2, '/atole/perception/cloud/completed', 2)
        self.marker_pub = self.create_publisher(MarkerArray, '/atole/perception/markers', 2)
        ActionServer(self, EstimatePods, '/atole/perception/estimate_pods', execute_callback=self._execute,
                     goal_callback=lambda _g: GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT,
                     cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=self.group)
        self.create_service(Trigger, '/atole/perception/pod_pose/warmup', self._srv_warmup, callback_group=self.group)
        self.get_logger().info('pod_pose_node listo')

    # ───────────────────────── configuración ─────────────────────────
    def _completion_args(self, eih):
        if not self.config.wait_ready():
            raise RuntimeError('sin /atole/config')
        c = self.config.section('Perception/Completion')
        return dict(python_exe=c.get('PythonExe', ''), worker=c.get('Worker', ''),
                    ckpt=c.get('EihCkpt' if eih else 'EthCkpt', ''),
                    seed=int(float(c.get('Seed', 0))), timeout=float(c.get('TimeoutS', 120)))

    # ───────────────────────── 1. captura sincronizada ─────────────────────────
    def _capture(self, cam, timeout=CAPTURE_TIMEOUT_S):
        base = f'/atole/zed/{cam}'
        topics = {'rgb': (Image, 'rgb/color/rect/image'), 'depth': (Image, 'depth/depth_registered'),
                  'conf': (Image, 'confidence/confidence_map'), 'info': (CameraInfo, 'rgb/color/rect/camera_info')}
        frames, done, lock = {}, threading.Event(), threading.Lock()

        def store(key, msg):
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            with lock:
                if done.is_set():
                    return
                frames.setdefault(stamp, {})[key] = msg
                if len(frames[stamp]) == len(topics):
                    frames['result'] = frames[stamp]
                    done.set()
                elif len(frames) > 6:                      # descarta stamps viejos incompletos
                    frames.pop(min(k for k in frames if k != 'result'))
        subs = [self.create_subscription(t, f'{base}/{name}', lambda m, k=key: store(k, m), SENSOR_QOS,
                                         callback_group=self.group) for key, (t, name) in topics.items()]
        try:
            if not done.wait(timeout):
                seen = sorted({k for f in frames.values() for k in f})
                raise TimeoutError(f'no llegó un frame completo de {base} en {timeout:.0f} s (llegó: {seen or "nada"})')
            return frames['result']
        finally:
            for s in subs:
                self.destroy_subscription(s)

    def _detect(self, cam, image):
        if not self.detect_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('detector_node no responde (/atole/perception/detect_once)')
        done = threading.Event()
        future = self.detect_client.call_async(DetectOnce.Request(camera=cam, include_masks=True, image=image))
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(DETECT_TIMEOUT_S):
            raise TimeoutError(f'detect_once sin respuesta en {DETECT_TIMEOUT_S:.0f} s')
        res = future.result()
        if not res.ok:
            raise RuntimeError(f'detector: {res.message}')
        return res.detections

    def _base_from_camera(self, cam, stamp):
        frame = f'{cam}_calibrated_optical'
        when = Time.from_msg(stamp) if cam == 'cam2' else Time()     # cam2 se mueve con el robot
        try:
            t = self.tf_buffer.lookup_transform(BASE, frame, when, timeout=Duration(seconds=1.0))
        except Exception as e:
            raise RuntimeError(f'sin TF {BASE} ← {frame} ({e}); ¿calibración de {cam} válida en Config.xml?')
        return transform_to_matrix(t.transform)

    # ───────────────────────── acción ─────────────────────────
    def _execute(self, gh):
        self.busy = True
        try:
            return self._estimate(gh)
        except Exception as e:
            self.get_logger().error(f'estimate_pods: {type(e).__name__}: {e}')
            gh.abort()
            return EstimatePods.Result(ok=False, message=f'{type(e).__name__}: {e}')
        finally:
            self.busy = False

    def _feedback(self, gh, stage, done=0, total=0):
        gh.publish_feedback(EstimatePods.Feedback(stage=stage, done=done, total=total))

    def _estimate(self, gh):
        g = gh.request
        t_start = time.monotonic()
        if not self.config.wait_ready():
            raise RuntimeError('sin /atole/config')
        cam = g.camera or ('cam2' if g.eih else self.config.get('Cameras/SelectedEtH', 'cam1'))
        eih = g.eih or cam == 'cam2'

        if len(g.cloud.data):
            pods, partials, T, stamp = self._from_cloud(g)
        else:
            pods, partials, T, stamp = self._from_camera(gh, g, cam, eih)
        R, t = T[:3, :3], T[:3, 3]
        to_base = lambda p: np.asarray(p, np.float64) @ R.T + t      # (3,) o (N, 3)
        n_exp = np.array([g.expected_axis.x, g.expected_axis.y, g.expected_axis.z])
        n_exp_cam = R.T @ n_exp / np.linalg.norm(n_exp) if eih and np.linalg.norm(n_exp) > 1e-9 else None

        # ── 4. completion → superquadric → PCA ──
        todo = [p for p in pods if p.status == 'ok']
        args = self._completion_args(eih)
        completed = {}
        for i, pod in enumerate(todo):
            if gh.is_cancel_requested:
                gh.canceled()
                return EstimatePods.Result(ok=False, message='cancelado')
            t0 = time.monotonic()
            try:
                self._feedback(gh, 'completion', i, len(todo))
                pre, _stats = preprocess_for_completion(partials[pod.id])
                if pre.shape[0] < 10:
                    raise RuntimeError(f'quedan {pre.shape[0]} puntos tras el preproceso')
                dense, _meta = self.pool.complete(pre, **args)
                self._feedback(gh, 'superquadric', i, len(todo))
                sq = fit_and_sample(dense, SQ_EIH if eih else None)
                if not sq['success']:
                    raise RuntimeError(f'superquadric: {sq["error_msg"]}')
                self._feedback(gh, 'pca', i, len(todo))
                r = estimate_pose(sq['surface_xyz'])
                if not r.success:
                    raise RuntimeError(f'PCA: {r.error_msg}')
            except Exception as e:
                pod.status = f'error: {e}'
                self.get_logger().warn(f'pod {pod.id}: {pod.status}')
                continue
            base, tip, axis, quat = r.grasp_position, r.tip_position, r.grasp_normal, r.quaternion
            if n_exp_cam is not None and float(np.dot(axis, n_exp_cam)) < 0:
                # Regla EiH de PozoleV3 (_eih_correction_estimate_sync): el signo lo decide el eje previsto,
                # no la regla "Y hacia abajo" de la cámara fija.
                base, tip, axis = tip, base, -axis
                quat = Rotation.align_vectors([axis], [[0.0, 0.0, 1.0]])[0].as_quat()
            completed[pod.id] = dense
            pod.n_completed = len(dense)
            pod.grasp_bottom = point(to_base(base))
            pod.tip = point(to_base(tip))
            pod.grasp_midpoint = point(to_base((base + tip) / 2.0))
            pod.centroid = point(to_base(r.position))
            pod.axis = vector(R @ axis)
            pod.orientation = quaternion((Rotation.from_matrix(R) * Rotation.from_quat(quat)).as_quat())
            pod.length_m = float(sq['metrics'].get('length_mm', 0.0)) / 1000.0
            pod.width_m = float(r.width_m)
            pod.pose_confidence = float(r.confidence)
            self.get_logger().info(f'pod {pod.id} ({pod.score:.2f}): {pod.n_partial} → {len(pre)} → {pod.n_completed} pts, '
                                   f'conf {r.confidence:.2f}, {1000 * (time.monotonic() - t0):.0f} ms')

        # ── salida ──
        out = PodArray(camera=cam, pods=pods)
        out.header.stamp, out.header.frame_id = stamp, BASE
        self.pods_pub.publish(out)
        self._publish_clouds(pods, partials, completed, to_base, stamp)
        result = EstimatePods.Result(ok=True, pods=out, camera_pose=matrix_to_pose(T))
        if g.return_clouds:     # una nube por pod, en el mismo orden que pods (vacía si no hay)
            empty = np.zeros((0, 3))
            result.partial_clouds = [cloud_msg(to_base(partials.get(q.id, empty)), BASE, stamp) for q in pods]
            result.completed_clouds = [cloud_msg(to_base(completed.get(q.id, empty)), BASE, stamp) for q in pods]
        n_ok = sum(p.status == 'ok' for p in pods)
        result.message = (f'{cam}: {len(pods)} detecciones, {n_ok} con pose, '
                          f'{sum(p.status == "out_of_workspace" for p in pods)} fuera del área de trabajo '
                          f'({time.monotonic() - t_start:.1f} s)')
        self.get_logger().info(result.message)
        gh.succeed()
        return result

    def _from_cloud(self, g):
        """Un solo pod a partir de la nube dada (marco óptico de la cámara en cloud_camera_pose)."""
        pts = cloud_to_array(g.cloud).astype(np.float32)
        T = pose_to_matrix(g.cloud_camera_pose)
        pod = Pod(id=0, detection_id=-1, class_name='pod', status='ok', in_workspace=True, n_partial=len(pts))
        if len(pts):
            pod.centroid = point(T[:3, :3] @ pts.mean(axis=0) + T[:3, 3])
        else:
            pod.status = 'error: nube vacía'
        return [pod], {0: pts}, T, self.get_clock().now().to_msg()

    def _from_camera(self, gh, g, cam, eih):
        self._feedback(gh, 'capture')
        frame = self._capture(cam)
        stamp = frame['depth'].header.stamp
        T = self._base_from_camera(cam, stamp)
        R, t = T[:3, :3], T[:3, 3]
        to_base = lambda p: np.asarray(p, np.float64) @ R.T + t

        self._feedback(gh, 'detect')
        dets = self._detect(cam, frame['rgb'])
        depth = image_to_array(frame['depth'])
        conf = image_to_array(frame['conf'])
        k = list(frame['info'].k)
        h, w = depth.shape
        cc = self.config.section('Perception/Cloud')
        ws = self.config.section('Perception/Workspace')
        f = lambda d, key, default: float(d.get(key, default))

        # ── 3. nube parcial + área de trabajo ──
        pods, partials = [], {}
        for det in dets.detections:
            self._feedback(gh, 'cloud', len(pods), len(dets.detections))
            pod = Pod(id=len(pods), detection_id=det.id, class_name=det.class_name, score=det.score)
            mask = mask_cloud.full_mask(image_to_array(det.mask), det.mask_xy, dets.image_height, dets.image_width)
            refined = mask_cloud.refine_mask(mask_cloud.align_mask(mask, h, w), conf, depth,
                                             erode_px=int(f(cc, 'MaskErodePx', 4)), conf_thr=f(cc, 'ConfThr', 50),
                                             grad_thr=f(cc, 'DepthGradThrM', 0.03))
            # EtH: tope de puntos (stride) como _extract_pc_frozen; EiH: todos, como _eih_extract_mask_points.
            pts = mask_cloud.object_points(depth, k, refined, 10 ** 9 if eih else int(f(cc, 'MaxPoints', 20000)))
            pod.n_partial = len(pts)
            if len(pts):
                c = to_base(pts.mean(axis=0))
                pod.centroid = point(c)
                reach = float(np.linalg.norm(c))
                pod.in_workspace = bool(f(ws, 'MinReachM', 0.0) <= reach <= f(ws, 'MaxReachM', 99.0)
                                        and f(ws, 'MinZ', -99.0) <= c[2] <= f(ws, 'MaxZ', 99.0))
                pod.in_workspace = pod.in_workspace or eih          # en EiH manda la asociación con la referencia
                pod.status = 'ok' if pod.in_workspace else 'out_of_workspace'
                partials[pod.id] = pts
            else:
                pod.status = 'error: la máscara no deja puntos válidos'
            pods.append(pod)

        if eih and g.gate_m > 0:        # EiH: solo el pod que corresponde al grasp previsto
            ref = np.array([g.reference.x, g.reference.y, g.reference.z])
            cands = [p for p in pods if p.status == 'ok' and p.n_partial >= EIH_MIN_POINTS]
            best = min(cands, key=lambda p: np.linalg.norm(np.array([p.centroid.x, p.centroid.y, p.centroid.z]) - ref),
                       default=None)
            if best is not None and np.linalg.norm(np.array([best.centroid.x, best.centroid.y, best.centroid.z]) - ref) > g.gate_m:
                best = None
            for p in pods:
                if p.status == 'ok' and p is not best:
                    p.status = 'not_selected'

        return pods, partials, T, stamp

    def _publish_clouds(self, pods, partials, completed, to_base, stamp):
        for pub, clouds in ((self.partial_pub, partials), (self.completed_pub, completed)):
            xyz = to_base(np.vstack(list(clouds.values()))) if clouds else np.zeros((0, 3))
            ids = np.concatenate([np.full(len(v), pid) for pid, v in clouds.items()]) if clouds else np.zeros(0)
            pub.publish(cloud_msg(xyz, BASE, stamp, ids))
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for p in pods:
            if p.status == 'ok':
                arrow = Marker(ns='pod_axis', id=p.id, type=Marker.ARROW, action=Marker.ADD, points=[p.grasp_bottom, p.tip])
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.02
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.67, 0.0, 1.0
                grasp = Marker(ns='pod_grasp', id=p.id, type=Marker.SPHERE, action=Marker.ADD)
                grasp.pose.position = p.grasp_bottom
                grasp.pose.orientation.w = 1.0
                grasp.scale.x = grasp.scale.y = grasp.scale.z = 0.025
                grasp.color.g, grasp.color.b, grasp.color.a = 0.9, 1.0, 1.0
                markers.markers += [arrow, grasp]
            text = Marker(ns='pod_label', id=p.id, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                          text=f'{p.id} · {p.score:.2f}' + ('' if p.status == 'ok' else f' · {p.status.split(":")[0]}'))
            text.pose.position = p.centroid
            text.pose.position.z += 0.08
            text.pose.orientation.w = 1.0
            text.scale.z = 0.04
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            markers.markers.append(text)
        for m in markers.markers:
            m.header.frame_id, m.header.stamp = BASE, stamp
        self.marker_pub.publish(markers)

    # ───────────────────────── warm-up ─────────────────────────
    def _srv_warmup(self, _req, res):
        # Cubo de 12³ puntos de 6 cm a 0.4 m (el mismo que usaba PozoleV3 para precalentar el EiH).
        g = np.linspace(-0.03, 0.03, 12, dtype=np.float32)
        cube = np.stack(np.meshgrid(g, g, g, indexing='ij'), -1).reshape(-1, 3)
        cube[:, 2] += 0.4
        parts = []
        try:
            for eih in (False, True):
                t0 = time.monotonic()
                self.pool.complete(cube, **self._completion_args(eih))
                parts.append(f'{"EiH" if eih else "EtH"} {time.monotonic() - t0:.1f} s')
        except Exception as e:
            res.success, res.message = False, f'warm-up de AdaPoinTr falló: {e}'
            self.get_logger().error(res.message)
            return res
        res.success, res.message = True, 'AdaPoinTr ' + ' · '.join(parts)
        self.get_logger().info(f'warm-up: {res.message}')
        return res

    def destroy_node(self):
        self.pool.shutdown()
        super().destroy_node()


def main(args=None):
    run_node(PodPoseNode, args)


if __name__ == '__main__':
    main()
