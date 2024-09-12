#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3, Twist, Point
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformStamped
import math

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')
        
        # Inicializar el broadcaster TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Suscribirse a los tópicos quaternion_topic, velocity_topic y position_topic
        self.quaternion_subscription = self.create_subscription(
            Quaternion,
            'quaternion_topic',
            self.quaternion_callback,
            10)
        
        self.velocity_subscription = self.create_subscription(
            Vector3,
            'velocity_topic',
            self.velocity_callback,
            10)

        self.position_subscription = self.create_subscription(
            Vector3,
            'vector3_topic',
            self.position_callback,
            10)
        
        # Crear publicador para el tópico de odometría
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        
        # Variables para almacenar los datos recibidos
        self.quaternion = None
        self.velocity = None
        self.position = None
        self.prev_quaternion = None
        self.prev_time = None
        
        # Configurar temporizador para publicar datos a intervalos regulares
        self.timer = self.create_timer(0.1, self.publish_odom)  # Publicar a 10 Hz

    def quaternion_callback(self, msg):
        self.quaternion = msg
        #self.get_logger().info(f'Received Quaternion: x={msg.x}, y={msg.y}, z={msg.z}, w={msg.w}')

    def velocity_callback(self, msg):
        self.velocity = msg
        #self.get_logger().info(f'Received Velocity: linear={msg.x}, angular={msg.z}')
        
    def position_callback(self, msg):
        self.position = msg
        #self.get_logger().info(f'Received Position: x={msg.x}, y={msg.y}, z={msg.z}')
    '''
    def rotate_quaternion_by_90_degrees(self, quaternion):
        # Cuaternión de rotación para una rotación de 90 grados alrededor del eje z
        rotation_quaternion = Quaternion()
        rotation_quaternion.x = 0.0
        rotation_quaternion.y = 0.0
        rotation_quaternion.z = math.sin(math.radians(90) / 2)
        rotation_quaternion.w = math.cos(math.radians(90) / 2)
        
        # Multiplicar el cuaternión actual por el cuaternión de rotación
        rotated_quaternion = Quaternion()
        rotated_quaternion.x = quaternion.w * rotation_quaternion.x + quaternion.x * rotation_quaternion.w + quaternion.y * rotation_quaternion.z - quaternion.z * rotation_quaternion.y
        rotated_quaternion.y = quaternion.w * rotation_quaternion.y - quaternion.x * rotation_quaternion.z + quaternion.y * rotation_quaternion.w + quaternion.z * rotation_quaternion.x
        rotated_quaternion.z = quaternion.w * rotation_quaternion.z + quaternion.x * rotation_quaternion.y - quaternion.y * rotation_quaternion.x + quaternion.z * rotation_quaternion.w
        rotated_quaternion.w = quaternion.w * rotation_quaternion.w - quaternion.x * rotation_quaternion.x - quaternion.y * rotation_quaternion.y - quaternion.z * rotation_quaternion.z
        
        # Normalizar el cuaternión resultante
        magnitude = math.sqrt(rotated_quaternion.x**2 + rotated_quaternion.y**2 + rotated_quaternion.z**2 + rotated_quaternion.w**2)
        rotated_quaternion.x /= magnitude
        rotated_quaternion.y /= magnitude
        rotated_quaternion.z /= magnitude
        rotated_quaternion.w /= magnitude
        
        return rotated_quaternion
    '''
    def publish_odom(self):
        if self.quaternion is not None and self.velocity is not None and self.position is not None:
            current_time = self.get_clock().now()  # Mover la asignación de current_time aquí
            
            if self.prev_quaternion is not None and self.prev_time is not None:
                # Calcular la diferencia entre los cuaterniones actuales y los anteriores
                diff_quaternion = self.calculate_difference(self.quaternion, self.prev_quaternion)
                
                # Calcular el tiempo transcurrido desde la última actualización
                delta_time = (current_time - self.prev_time).nanoseconds / 1e9  # Convertir a segundos
                
                # Calcular la velocidad angular a partir de la diferencia de cuaterniones y el tiempo transcurrido
                angular_velocity = self.quaternion_to_angular_velocity(diff_quaternion, delta_time)
            else:
                angular_velocity = 0.0  # Si es la primera vez, asignar cero
                
            # Crear mensaje de odometría
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"

            # Rellenar la orientación y la posición
            #current_quaternion = self.quaternion
            #odom.pose.pose.orientation = self.rotate_quaternion_by_90_degrees(current_quaternion)
            odom.pose.pose.orientation = self.quaternion
            odom.pose.pose.position = Point(x=self.position.x, y=self.position.y, z=self.position.z)

            # Crear un nuevo objeto Twist y asignar los valores de velocidad lineal y angular
            twist = Twist()
            twist.linear.x = self.velocity.x
            twist.linear.y = self.velocity.y
            twist.linear.z = self.velocity.z
            twist.angular.z = angular_velocity  # Asignar la velocidad angular calculada

            # Asignar el objeto Twist creado a odom.twist.twist
            odom.twist.twist = twist

            # Publicar el mensaje de odometría
            self.odom_publisher.publish(odom)
            #self.get_logger().info('Publishing Odometry')

            # Publicar la transformación TF
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.position.x
            t.transform.translation.y = self.position.y
            t.transform.translation.z = self.position.z
            t.transform.rotation = odom.pose.pose.orientation

            self.tf_broadcaster.sendTransform(t)

            # Actualizar el cuaternión y el tiempo previos para el siguiente cálculo
            self.prev_quaternion = self.quaternion
            self.prev_time = current_time

    def calculate_difference(self, q1, q2):
        # Calcular la diferencia entre dos cuaterniones
        diff = Quaternion()
        diff.w = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z
        diff.x = q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y
        diff.y = q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x
        diff.z = q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w

        return diff

    def quaternion_to_angular_velocity(self, diff_quaternion, delta_time):
        # Convierte la diferencia de cuaterniones en velocidad angular
        magnitude = math.sqrt(diff_quaternion.x**2 + diff_quaternion.y**2 + diff_quaternion.z**2)
        angular_velocity = magnitude / delta_time * 0.5
        return angular_velocity


def main(args=None):
    rclpy.init(args=args)
    node = OdomTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
