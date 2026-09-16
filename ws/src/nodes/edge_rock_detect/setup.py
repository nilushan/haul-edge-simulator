from setuptools import find_packages, setup

package_name = 'edge_rock_detect'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='edge_rock_detect',
    maintainer_email='dev@example.com',
    description='Rock detection node (LiDAR → /edge/lidar/rocks + alerts)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rock_detect_node = edge_rock_detect.rock_node:main',
        ],
    },
)
