from setuptools import find_packages, setup


package_name = 'fast_planner_common'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhr',
    maintainer_email='jhr@example.com',
    description='Shared map loading and path feasibility checking for FastPlanner.',
    license='MIT',
)
