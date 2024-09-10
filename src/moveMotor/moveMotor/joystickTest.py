import sys
import os
import pygame
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Añadir el directorio actual al PYTHONPATH
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot import Robot  # Ahora debería encontrar robot.py correctamente

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')
        self.publisher_ = self.create_publisher(String, 'joystick_states', 10)

    def publish_state(self, state):
        msg = String()
        msg.data = state
        self.publisher_.publish(msg)

def main():
    # Inicialización de Pygame
    pygame.init()

    # Inicialización del joystick
    pygame.joystick.init()

    # Comprobación de que hay al menos un joystick conectado
    if pygame.joystick.get_count() < 1:
        raise Exception("No se detectó ningún joystick. Conecta un mando USB e intenta de nuevo.")

    # Inicialización del primer joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Inicialización del robot
    robot = Robot()

    # Inicialización de ROS 2
    rclpy.init()

    joystick_publisher = JoystickPublisher()

    # Definición de los valores de velocidad
    speed = 0.5
    running = True

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                
                if event.type == pygame.JOYAXISMOTION:
                    # El eje 1 y el eje 3 generalmente corresponden a las palancas izquierda y derecha en la dirección vertical
                    left_stick_y = -joystick.get_axis(1)
                    right_stick_y = -joystick.get_axis(3)
                    
                    # Invirtiendo los valores para que hacia arriba sea positivo
                    left_speed = left_stick_y
                    right_speed = right_stick_y
                    
                    # Configuración de los motores
                    robot.set_motors(left_speed * speed, right_speed * speed)

                    # Publicar el estado del joystick
                    joystick_publisher.publish_state(f"Left Stick Y: {left_stick_y}, Right Stick Y: {right_stick_y}")

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 0:  # Suponiendo que el botón 0 es el botón A
                        robot.forward(speed)
                        joystick_publisher.publish_state("Button A pressed")
                    elif event.button == 1:  # Suponiendo que el botón 1 es el botón B
                        robot.backward(speed)
                        joystick_publisher.publish_state("Button B pressed")
                    elif event.button == 2:  # Suponiendo que el botón 2 es el botón X
                        robot.left(speed)
                        joystick_publisher.publish_state("Button X pressed")
                    elif event.button == 3:  # Suponiendo que el botón 3 es el botón Y
                        robot.right(speed)
                        joystick_publisher.publish_state("Button Y pressed")
                
                if event.type == pygame.JOYBUTTONUP:
                    if event.button in [0, 1, 2, 3]:  # Suponiendo botones A, B, X, Y
                        robot.stop()
                        joystick_publisher.publish_state(f"Button {event.button} released")

            # Opcional: agregar un pequeño retardo para no saturar la CPU
            sleep(0.01)
    finally:
        pygame.quit()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
