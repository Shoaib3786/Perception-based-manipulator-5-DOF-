from glob import glob
from setuptools import find_packages, setup

package_name = 'roboarm_proj1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shoaib Khan',
    maintainer_email='mohammad.shoaib3786@gmail.com',
    description='ROS 2 components for the perception-based manipulator project.',
    license='TODO: License declaration',
    entry_points={'console_scripts': ['simple_publisher = roboarm_proj1.simple_publisher:main', 'simple_subscriber = roboarm_proj1.simple_subscriber:main']},
)
