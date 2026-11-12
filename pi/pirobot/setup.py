from glob import glob
from setuptools import find_packages, setup

package_name = 'pirobot'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Swagat Kumar',
    maintainer_email='swagat.kumar@gmail.com',
    description='ROS2 Control Interface for PiRobot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_publisher = pirobot.keyboard_publisher:main',
            'motion_subscriber = pirobot.motion_subscriber:main',
            'picam_publisher = pirobot.picam_publisher:main',
            'calibrate_motors = pirobot.calibrate_motors:main',
            # short aliases kept from earlier versions
            'keypub = pirobot.keyboard_publisher:main',
            'motionsub = pirobot.motion_subscriber:main',
        ],
    },
)
