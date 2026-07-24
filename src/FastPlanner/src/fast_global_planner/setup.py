from glob import glob

from setuptools import find_packages, setup


package_name = 'fast_global_planner'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhr',
    maintainer_email='jhr@example.com',
    description='Standalone switchable 3D global planner with feasibility gating.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fast_global_planner_node = fast_global_planner.feasibility_integration_node:main',
            'fast_planner_feasibility_test = fast_global_planner.offline_feasibility_test:main',
        ],
    },
)
