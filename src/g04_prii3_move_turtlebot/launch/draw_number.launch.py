from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    draw_number_node = Node(
        package='g04_prii3_move_turtlebot',
        executable='draw_number',
        name='draw_number',
        output='screen'
    )

    return LaunchDescription([
        draw_number_node
    ])