from setuptools import setup

package_name = 'qube_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='PID‐controller for Qube',
    license='MIT',
    entry_points={
        'console_scripts': [
            'controller_node = qube_controller.controller:main',
        ],
    },
)