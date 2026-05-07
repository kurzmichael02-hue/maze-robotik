from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'maze_explorer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Kurz',
    maintainer_email='kurzmichael02@gmail.com',
    description='frontier exploration + slam config',
    license='MIT',
    entry_points={
        'console_scripts': [
            'frontier_explorer = maze_explorer.frontier_explorer:main',
            'path_executor = maze_explorer.path_executor:main',
        ],
    },
)
