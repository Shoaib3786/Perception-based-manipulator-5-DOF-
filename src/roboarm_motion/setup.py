from glob import glob
from setuptools import find_packages, setup

package_name = 'roboarm_motion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shoaib Khan',
    maintainer_email='mohammad.shoaib3786@gmail.com',
    description='ROS 2 components for the perception-based manipulator project.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['approach_node = roboarm_motion.approach_node:main']},
)
