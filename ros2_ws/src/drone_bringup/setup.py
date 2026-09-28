from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
        os.path.join("share", "drone_bringup", "launch"),
        glob("launch/*.launch.py"),
    ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maggen',
    maintainer_email='maggenfaggen@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
