from launch import LaunchDescription

from launch_ros.actions import Node

import os



def generate_launch_description():

    urdf_file_path = os.path.join(

        os.path.dirname(__file__), '../urdf/robot.urdf'

    )

    with open(urdf_file_path, 'r') as urdf_file:

        robot_description = urdf_file.read()



    return LaunchDescription([

        Node(

            package='robot_state_publisher',

            executable='robot_state_publisher',

            name='robot_state_publisher',

            parameters=[{'robot_description': robot_description}]

        ),

        Node(

            package='ros2_viz_demo',

            executable='joint_state_published',  # <- match the filename (without .py)

            name='joint_state_publisher',

            output='screen'

        )

    ])

