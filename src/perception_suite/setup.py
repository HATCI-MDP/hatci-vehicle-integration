"""
Package definition for perception_suite.

The console_scripts names start with digits so the stage order is visible in
`ros2 run` and in a process list. That is legal for an entry point even though
`import perception_suite.5_planner_node` is not, which is why the tests load
the node modules through importlib.
"""

from glob import glob

from setuptools import find_packages, setup

package_name = 'perception_suite'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shen6',
    maintainer_email='felixshn@umich.edu',
    description=('ROS 2 pipeline for off-road road segmentation, path planning '
                 'and control. Thin nodes; the algorithms live in the '
                 'offroad_autonomy library. See src/README.md.'),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'live_odom_node = perception_suite.live_odom_node:main',
            'odom_node = perception_suite.odom_node:main',
            '1_bag_node = perception_suite.1_bag_node:main',
            '1_video_node = perception_suite.1_video_node:main',
            '1_camera_node = perception_suite.1_camera_node:main',
            '2_preprocess_node = perception_suite.2_preprocess_node:main',
            '3_segmentation_node = perception_suite.3_segmentation_node:main',
            '4_postprocess_node = perception_suite.4_postprocess_node:main',
            '5_planner_node = perception_suite.5_planner_node:main',
            '6_visualizer_node = perception_suite.6_visualizer_node:main',
            '7_controller_node = perception_suite.7_controller_node:main',
        ],
    },
)
