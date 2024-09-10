#!/usr/bin/python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
import math

import sys
sys.path.append('src/tf_transform/tf_transform')
import pyrealsense2 as rs

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')

        # Inicializar la cámara Realsense
        self.pipe = rs.pipeline()
        self.cfg = rs.config()
        self.cfg.enable_stream(rs.stream.pose)
        self.pipe.start(self.cfg)
        
        # Crear publicadores para Vector3, Quaternion y Velocity
        self.vector_publisher = self.create_publisher(Vector3, 'vector3_topic', 10)
        self.quaternion_publisher = self.create_publisher(Quaternion, 'quaternion_topic', 10)
        self.velocity_publisher = self.create_publisher(Vector3, 'velocity_topic', 10)
        
        # Configurar temporizador para publicar datos a intervalos regulares
        self.timer = self.create_timer(0.05, self.publish_data)  # Publicar a 10 Hz
        
    def euler_to_quaternion(self, yaw, roll, pitch):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def quaternion_to_euler(self, q):
        # Convert a quaternion to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    def publish_data(self):
        frames = self.pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if pose:
            data = pose.get_pose_data()
            
            # Actualizar Vector3
            vector = Vector3()
            vector.x = -data.translation.z
            vector.y = -data.translation.x
            vector.z = data.translation.y
            
            # Actualizar Quaternion
            quaternion = Quaternion()
            quaternion.w = data.rotation.w
            quaternion.x = data.rotation.x
            quaternion.y = data.rotation.z
            quaternion.z = data.rotation.y
            
            # Convertir quaternion a ángulos de Euler
            roll, pitch, yaw = self.quaternion_to_euler(quaternion)
            quaternion=self.euler_to_quaternion(yaw, -pitch, -roll)
            
            # Actualizar Velocity
            velocity = Vector3()
            velocity.x = data.velocity.z
            velocity.y = data.velocity.x
            velocity.z = data.velocity.y
            
            # Publicar Vector3, Quaternion y Velocity
            self.vector_publisher.publish(vector)
            self.quaternion_publisher.publish(quaternion)
            self.velocity_publisher.publish(velocity)
            
            self.get_logger().info(f'Publishing Vector3: x={vector.x}, y={vector.y}, z={vector.z}')
            self.get_logger().info(f'Publishing Quaternion: x={quaternion.x}, y={quaternion.y}, z={quaternion.z}, w={quaternion.w}')
            self.get_logger().info(f'Publishing Velocity: x={velocity.x}, y={velocity.y}, z={velocity.z}')

    def __del__(self):
        self.pipe.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
