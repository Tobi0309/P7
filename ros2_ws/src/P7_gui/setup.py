from setuptools import find_packages, setup

package_name = 'P7_gui'

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
    maintainer='rasmus',
    maintainer_email='rdkr23@student.aau.dk',
    description='Operator GUI for the P7 drone stack.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'p7_gui = P7_gui.main:main',
        ],
    },
)
