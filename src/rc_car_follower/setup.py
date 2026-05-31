from setuptools import find_packages, setup

package_name = 'rc_car_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='ccojcp312@gmail.com',
    description='Python nodes for detecting, approaching, and following an RC car.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'tripod_yolo_node = rc_car_follower.tripod_yolo_node:main',
            'pixel_to_map_node = rc_car_follower.pixel_to_map_node:main',
            'approach_planner_node = rc_car_follower.approach_planner_node:main',
            'oakd_yolo_depth_node = rc_car_follower.oakd_yolo_depth_node:main',
            'follow_controller_node = rc_car_follower.follow_controller_node:main',
            'nav2_target_tracker_node = rc_car_follower.nav2_target_tracker_node:main',
            'supervisor_node = rc_car_follower.supervisor_node:main',
            'tripod_trigger_node = rc_car_follower.tripod_trigger_node:main',
            'webcam_car_test = rc_car_follower.webcam_car_test:main',
        ],
    },
)
