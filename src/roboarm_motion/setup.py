from glob import glob
from setuptools import find_packages, setup
setup(
    name='roboarm_motion', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/roboarm_motion']),
        ('share/roboarm_motion', ['package.xml', 'README.md']),
        ('share/roboarm_motion/launch', glob('launch/*.launch.py')),
        ('share/roboarm_motion/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Shoaib Khan', maintainer_email='mohammad.shoaib3786@gmail.com',
    description='Perception to MoveIt Gazebo approach demo', license='BSD-3-Clause',
    entry_points={'console_scripts': [
        'approach_node = roboarm_motion.approach_node:main',
        'keyboard_teleop = roboarm_motion.keyboard_teleop:main',
    ]},
)
