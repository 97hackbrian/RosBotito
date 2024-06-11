import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

def euler_to_quaternion(yaw, pitch, roll):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = TransformStamped().transform.rotation
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)
        self.x_value = 1.0
        self.y_value = 1.0
        self.z_value = 0.0
        self.yaw = -1.0
        self.pitch = 0.0
        self.roll = 0.0

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x_value
        transform.transform.translation.y = self.y_value
        transform.transform.translation.z = self.z_value
        transform.transform.rotation = euler_to_quaternion(self.yaw, self.pitch, self.roll)

        self.br.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
