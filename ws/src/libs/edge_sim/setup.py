from setuptools import find_packages, setup

package_name = 'edge_sim'

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
    maintainer='edge_sim',
    maintainer_email='dev@example.com',
    description='Haul-truck sensor simulation library',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'generate_stream = edge_sim.tools.generate_stream:main',
            'generate_offline = edge_sim.tools.generate_offline:main',
        ],
    },
)
