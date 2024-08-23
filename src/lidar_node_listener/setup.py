from setuptools import find_packages, setup

package_name = 'lidar_node_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skemp32',
    maintainer_email='samkemp1996@gmail.com',
    description='Listens to gazebo lidar and plots data',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'listener = lidar_node_listener.subscriber_member_function:main',
        ],
    },
)
