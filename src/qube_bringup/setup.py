from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qube_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anton-knivsfla',
    maintainer_email='antonkniv@gmail.com',
    description='Bringup package for Qube using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)