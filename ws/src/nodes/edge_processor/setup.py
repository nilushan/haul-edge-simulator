from setuptools import find_packages, setup

package_name = 'edge_processor'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='edge_processor',
    maintainer_email='dev@example.com',
    description='Example haul-edge processor node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'processor_node = edge_processor.processor_node:main',
        ],
    },
)
