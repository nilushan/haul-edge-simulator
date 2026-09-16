from setuptools import find_packages, setup
from glob import glob

package_name = 'edge_sensor_sim'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*')),
        (
            f'share/{package_name}/static',
            glob('edge_sensor_sim/adapters/web/static/*'),
        ),
    ],
    install_requires=['setuptools', 'numpy', 'aiohttp'],
    zip_safe=True,
    maintainer='edge_sensor_sim',
    maintainer_email='dev@example.com',
    description='Haul-truck edge sensors: domain models, stream hub, ROS bus, web viz',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ROS nodes
            'sensor_suite_node = edge_sensor_sim.adapters.ros.sensor_source_node:main',
            'processor_stub_node = edge_sensor_sim.adapters.ros.processor_stub_node:main',
            # Web
            'viz_server = edge_sensor_sim.adapters.web.hub_server:main',
            'viz_bus_server = edge_sensor_sim.adapters.web.bus_server:main',
            # Files / apps
            'generate_offline = edge_sensor_sim.adapters.files.generate_offline:main',
            'generate_stream = edge_sensor_sim.adapters.files.generate_stream:main',
            'stream_server = edge_sensor_sim.apps.stream_server:main',
        ],
    },
)
