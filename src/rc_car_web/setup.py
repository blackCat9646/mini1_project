from setuptools import find_packages, setup

package_name = 'rc_car_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='ccojcp312@gmail.com',
    description='Web command bridge nodes for the TurtleBot4 RC car mini project.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'web_command_node = rc_car_web.web_command_node:main',
            'web_map_node = rc_car_web.web_map_node:main',
            'web_pose_node = rc_car_web.web_pose_node:main',
        ],
    },
)
