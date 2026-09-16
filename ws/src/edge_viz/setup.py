from setuptools import find_packages, setup
from glob import glob

package_name = 'edge_viz'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/static', glob('edge_viz/static/*')),
    ],
    install_requires=['setuptools', 'numpy', 'aiohttp'],
    zip_safe=True,
    maintainer='edge_viz',
    maintainer_email='dev@example.com',
    description='Haul-edge web visualizer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'viz_from_hub = edge_viz.run_from_hub:main',
            'viz_from_bus = edge_viz.run_from_bus:main',
        ],
    },
)
