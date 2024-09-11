from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Lanza el nodo moveMotor
        ExecuteProcess(
            cmd=['ros2', 'run', 'moveMotor', 'MotorTopic'],
            output='screen'
        ),

        # Lanza el RPLIDAR
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rplidar_ros', 'rplidar.launch.py'],
            output='screen'
        ),

        # Publica la transformación tf
        ExecuteProcess(
            cmd=['ros2', 'run', 'tf_transform', 'Publish'],
            output='screen'
        ),
    ])
