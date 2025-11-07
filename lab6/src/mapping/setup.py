from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.xml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ee106a-agt',
    maintainer_email='kchilaka@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = mapping.mapping_node:main',
        ],
    },
)
