from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'soft_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='.'),  # current directory holds packages
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/URDF', ['URDF/URDF.urdf']),
        ('share/' + package_name + '/URDF/meshes', glob('URDF/meshes/*.stl')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='felix',
    maintainer_email='your_email@example.com',
    description='Soft gripper control package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'gripper_node = soft_gripper.main:main',
        ],
    },
)






