from setuptools import setup

package_name = 'auto_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='you@example.com',
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='A simple ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_sender_node = auto_robot.sender:main',
            'motor_receiver_node = auto_robot.receiver:main',
            'motor_test_node = auto_robot.sender_sine_wave_test:main',
            'can_motor_node = auto_robot.can_motor:main',
        ],
    },
)

