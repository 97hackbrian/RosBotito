import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import depthai as dai
import struct

class CloudPointsPublisher(Node):
    def __init__(self):
        super().__init__('cloud_points_publisher')

        # Crear pipeline de DepthAI
        self.pipeline = dai.Pipeline()
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        depth = self.pipeline.create(dai.node.StereoDepth)
        pointcloud = self.pipeline.create(dai.node.PointCloud)
        xOut = self.pipeline.create(dai.node.XLinkOut)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        monoLeft.out.link(depth.left)
        monoRight.out.link(depth.right)
        depth.depth.link(pointcloud.inputDepth)
        pointcloud.outputPointCloud.link(xOut.input)
        xOut.setStreamName("out")

        self.device = dai.Device(self.pipeline)

        # Definir el perfil de QoS a Best Effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=15
        )

        # Crear el publicador de ROS 2 para PointCloud2 con QoS
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud', qos_profile)
        self.timer = self.create_timer(0.05, self.publish_pointcloud)

    def publish_pointcloud(self):
        q = self.device.getOutputQueue(name="out", maxSize=4, blocking=False)
        inPointCloud = q.get()

        if inPointCloud:
            points = inPointCloud.getPoints().astype(np.float32)
            # Reducción de la cantidad de puntos
            reduced_points = self.reduce_points(points, factor=40)  # Factor de muestreo
            pointcloud_msg = self.create_pointcloud2(reduced_points)
            self.publisher_.publish(pointcloud_msg)
            self.get_logger().info(f'Publishing {len(reduced_points)} points')

    def reduce_points(self, points, factor):
        # Reducir la cantidad de puntos muestreando cada 'factor'-ésimo punto
        if factor < 1:
            factor = 1
        return points[::factor]

    def create_pointcloud2(self, points):
        msg = PointCloud2()
        msg.header.frame_id = "camera_frame"
        msg.height = 1
        msg.width = points.shape[0]

        # Asegurarse de que el campo de puntos es el correcto
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True  # Esto debe ser True si no hay puntos NaN

        # Asegúrate de que los datos se conviertan en bytes correctamente
        msg.data = np.array(points, dtype=np.float32).tobytes()

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CloudPointsPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
