"""Llama a /atole/perception/estimate_pods y muestra los pods (uso: estimate_pods.py [cámara] [--eih])."""
import sys
import time

import rclpy
from atole_interfaces.action import EstimatePods
from rclpy.action import ActionClient


def main():
    cam = next((a for a in sys.argv[1:] if not a.startswith('--')), '')
    rclpy.init()
    node = rclpy.create_node('estimate_pods_cli')
    client = ActionClient(node, EstimatePods, '/atole/perception/estimate_pods')
    if not client.wait_for_server(timeout_sec=10):
        sys.exit('sin servidor de estimate_pods')
    t0 = time.monotonic()
    stages = []
    goal = EstimatePods.Goal(camera=cam, eih='--eih' in sys.argv, return_clouds=False)
    fut = client.send_goal_async(goal, feedback_callback=lambda f: stages.append(
        f'{f.feedback.stage}{f"[{f.feedback.done}/{f.feedback.total}]" if f.feedback.total else ""}'))
    rclpy.spin_until_future_complete(node, fut)
    res_fut = fut.result().get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=300)
    r = res_fut.result().result
    print(f'ok={r.ok} · {r.message} · {time.monotonic() - t0:.1f} s en total')
    print('etapas:', ' → '.join(dict.fromkeys(s.split('[')[0] for s in stages)))
    for p in r.pods.pods:
        g, a = p.grasp_bottom, p.axis
        print(f'  pod {p.id:2d} score {p.score:.3f} [{p.status}] parcial {p.n_partial:5d} completada {p.n_completed:5d} '
              f'grasp ({g.x:+.4f}, {g.y:+.4f}, {g.z:+.4f}) eje ({a.x:+.3f}, {a.y:+.3f}, {a.z:+.3f}) '
              f'L {p.length_m * 1000:.0f} mm W {p.width_m * 1000:.0f} mm conf {p.pose_confidence:.2f}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
