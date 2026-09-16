from setuptools import setup
from glob import glob

package_name = 'edge_bringup'

setup(
    name=package_name,
    version='0.3.0',
    packages=["edge_bringup"],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edge_bringup',
    maintainer_email='dev@example.com',
    description='Haul-edge bringup',
    license='Apache-2.0',
)
