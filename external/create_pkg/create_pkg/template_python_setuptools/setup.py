import os
from glob import glob
from setuptools import setup

package_name = 'hello_world'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.param.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MAINTAINER_NAME',
    maintainer_email='MAINTAINER_EMAIL',
    description='INSERT_DESCRIPTION',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hello_world_node = hello_world.hello_world_node:main'
        ],
    },
)
