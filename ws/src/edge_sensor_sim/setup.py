from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'edge_sensor_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*')),
        (f'share/{package_name}/static', glob('edge_sensor_sim/static/*')),
    ],
    install_requires=['setuptools', 'numpy', 'aiohttp'],
    zip_safe=True,
    maintainer='edge_sensor_sim',
    maintainer_email='dev@example.com',
    description='Synthetic haul-truck edge sensor suite for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_suite_node = edge_sensor_sim.sensor_suite_node:main',
            'processor_stub_node = edge_sensor_sim.processor_stub_node:main',
            'generate_offline = edge_sensor_sim.generate_offline:main',
            'generate_stream = edge_sensor_sim.generate_stream:main',
            'viz_server = edge_sensor_sim.viz_server:main',
            'viz_bus_server = edge_sensor_sim.viz_bus_server:main',
            'stream_server = edge_sensor_sim.stream_server:main',
        ],
    },
)
