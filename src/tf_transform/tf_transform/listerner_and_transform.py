import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros import TransformBroadcaster

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_transform)
        
        # Inicializar suscriptores
        self.subscription_quat = self.create_subscription(
            Quaternion,
            'quaternion_topic',
            self.quaternion_callback,
            10
        )
        
        self.subscription_vec = self.create_subscription(
            Vector3,
            'vector3_topic',
            self.vector_callback,
            10
        )
        
        # Inicializar variables para almacenar los datos
        self.current_quaternion = Quaternion()
        self.current_vector = Vector3()
        
    def quaternion_callback(self, msg):
        self.current_quaternion = msg

    def vector_callback(self, msg):
        self.current_vector = msg

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        
        transform.transform.translation.x = self.current_vector.x
        transform.transform.translation.y = self.current_vector.y
        transform.transform.translation.z = self.current_vector.z
        transform.transform.rotation = self.current_quaternion
        
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
