from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pencilnet_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='krishna@todo.todo',
    description='PencilNet gate-detection dataset generator (teleport + depth occlusion)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'data_generator = pencilnet_ros.data_generator_node:main',
        ],
    },
)
