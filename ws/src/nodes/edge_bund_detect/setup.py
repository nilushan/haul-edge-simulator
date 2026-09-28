from setuptools import find_packages, setup

package_name = 'edge_bund_detect'

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
    maintainer='Nilushan Silva',
    maintainer_email='nilushan.silva@gmail.com',
    description='Bund/berm detection node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bund_detect_node = edge_bund_detect.bund_node:main',
        ],
    },
)
