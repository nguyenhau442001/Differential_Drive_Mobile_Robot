from setuptools import setup
import os
from glob import glob

package_name = 'mobile_robot_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nguyen Ngoc Hau',
    maintainer_email='nguyenhau442001@gmail.com',
    description='Provides teleoperation using keyboard for Mobile Robot.',
    license='BSD',
    entry_points={
        'console_scripts': [
            'mobile_robot_teleop_key = mobile_robot_teleop.mobile_robot_teleop_key:main',
        ],
    },
)
