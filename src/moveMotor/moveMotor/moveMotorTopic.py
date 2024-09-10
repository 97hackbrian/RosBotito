import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Añadir el directorio actual al PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot import Robot  # Ahora debería encontrar robot.py correctamente

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Inicialización del robot
        self.robot = Robot()

    def listener_callback(self, msg):
        # Extraer las velocidades lineales y angulares del mensaje
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Asumimos que el robot usa dos motores, y los controlamos en base a las velocidades lineales y angulares
        left_speed = linear_x - angular_z
        right_speed = linear_x + angular_z

        # Configuración de los motores
        self.robot.set_motors(left_speed, right_speed)

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
