from setuptools import setup, find_packages

import os

from glob import glob



package_name = 'ros2_viz_demo'



setup(

    name=package_name,

    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[

        ('share/ament_index/resource_index/packages',

         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),

    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Emin',

    maintainer_email='emin@example.com',

    description='URDF + JointStatePublisher demo for RViz visualization',

    license='MIT',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [

            'circular_point = ros2_viz_demo.circular_point:main',

            'marker_array = ros2_viz_demo.marker_array:main',

            'moving_cube = ros2_viz_demo.moving_cube:main',

            'pose_arrow = ros2_viz_demo.pose_arrow:main',

            'curved_trail_node = ros2_viz_demo.curved_trail_node:main',

            'joint_state_Published = ros2_viz_demo.joint_state_Published:main',

            'pose_and_marker_array_publisher = ros2_viz_demo.pose_and_marker_array_publisher:main',

        ],

    },

)

