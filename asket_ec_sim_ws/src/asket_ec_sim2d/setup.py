import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'asket_ec_sim2d'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Asket EC Engineering Club',
    maintainer_email='asket_ec@engineering.club',
    description='Simulateur 2D léger pour le bateau Asket EC (Python pur, sans Gazebo)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = asket_ec_sim2d.simulator_node:main',
        ],
    },
)
