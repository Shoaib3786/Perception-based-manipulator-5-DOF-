from glob import glob
from setuptools import find_packages, setup

package_name = 'perception_roboarm'

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
    entry_points={'console_scripts': ['hsvTracking_node = perception_roboarm.hsv_color_tracking:main', 'segmentation_node = perception_roboarm.segmentation:main', 'depth_node = perception_roboarm.depth_detection:main']},
)
