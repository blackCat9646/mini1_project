from setuptools import find_packages, setup

package_name = 'ros2_homework_examples'

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
    description='Simple ROS2 publisher and subscriber homework examples.',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=[
        'ros2_homework_examples/2_0_a_image_publisher.py',
        'ros2_homework_examples/2_0_b_image_subscriber.py',
        'ros2_homework_examples/2_0_c_data_publisher.py',
        'ros2_homework_examples/2_0_d_data_subscriber.py',
    ],
)
