from setuptools import find_packages, setup

package_name = 'roboarm_proj1'

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shoaib-ubuntu',
    maintainer_email='shoaib-ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_Msgpublisher=roboarm_proj1.simple_publisher:main',
            'my_Msgsubscriber=roboarm_proj1.simple_subscriber:main',
        ],
    },
)
