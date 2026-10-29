from setuptools import find_packages, setup

package_name = 'P7_sim_bridge'

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
    maintainer='rasmus',
    maintainer_email='rdkr23@student.aau.dk',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'isaac_lidar_to_livox_lidar = P7_sim_bridge.isaac_lidar_to_livox_lidar:main',
            'joy_to_drone_velocities = P7_sim_bridge.joy_to_drone_velocities:main',
        ],
    },
)
