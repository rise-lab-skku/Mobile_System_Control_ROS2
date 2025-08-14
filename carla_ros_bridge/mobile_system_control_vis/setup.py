import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'mobile_system_control_vis'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py', recursive=True)),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz', recursive=True)),
        (os.path.join('share', package_name, 'track'), glob('track/*.csv', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kyuhun',
    maintainer_email='kyuhun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobile_system_control_vis = mobile_system_control_vis.mobile_system_control_vis:main',
        ],
    },
)
