"""cloud_fusion_node — fusión de las vistas EiH (FUSED_2VIEW y PPP).

/atole/perception/fuse_views (FuseViews): pasa cada nube (base_link) al marco de la cámara de
referencia (la última vista, como PozoleV3) y las fusiona con voxel-merge (fusion.py, EIH/FuseVoxelM).
El resultado vuelve en ese marco óptico, listo para estimar la pose (EstimatePods con `cloud`), y se
publica en base_link en /atole/perception/cloud/fused para verlo en Lichtblick.
"""
from atole_interfaces.srv import FuseViews
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from atole_common.config_view import ConfigView
from atole_common.stubs import run_node
from atole_perception.fusion import to_frame, voxel_merge
from atole_perception.ros_utils import cloud_msg, cloud_to_array, pose_to_matrix

FUSED_FRAME = 'fused_camera'


class CloudFusionNode(Node):

    def __init__(self):
        super().__init__('cloud_fusion_node')
        self.config = ConfigView(self)
        self.fused_pub = self.create_publisher(PointCloud2, '/atole/perception/cloud/fused', 2)
        self.create_service(FuseViews, '/atole/perception/fuse_views', self._srv_fuse)
        self.get_logger().info('cloud_fusion_node listo')

    def _srv_fuse(self, req, res):
        if not req.clouds:
            res.ok, res.message = False, 'no hay nubes que fusionar'
            return res
        voxel = req.voxel_m or self.config.get_float('EIH/FuseVoxelM', 0.003)
        t_ref = pose_to_matrix(req.target_camera_pose)
        views = [to_frame(cloud_to_array(c), t_ref) for c in req.clouds]
        fused = voxel_merge(views, voxel)
        stamp = self.get_clock().now().to_msg()
        res.fused = cloud_msg(fused, FUSED_FRAME, stamp)
        res.n_fused = len(fused)
        self.fused_pub.publish(cloud_msg(fused @ t_ref[:3, :3].T + t_ref[:3, 3], 'base_link', stamp))
        res.ok = True
        res.message = f'{" + ".join(str(len(v)) for v in views)} puntos → {len(fused)} (vóxel {voxel * 1000:.1f} mm)'
        self.get_logger().info(f'fuse_views: {res.message}')
        return res


def main(args=None):
    run_node(CloudFusionNode, args)


if __name__ == '__main__':
    main()
