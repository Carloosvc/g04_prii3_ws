from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Inicia el simulador turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # Inicia nuestro nodo que dibuja el 04
        Node(
            package='g04_prii3_turtlesim',
            executable='dibujo04',
            name='dibujo04'
        )

    ])
