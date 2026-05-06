from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'maze_planners'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Kurz',
    maintainer_email='kurzmichael02@gmail.com',
    description='path planning algorithms + benchmark',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'benchmark = maze_planners.benchmark:main',
            'visualize = maze_planners.visualize:main',
        ],
    },
)
