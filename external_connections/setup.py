from setuptools import setup
import os
from glob import glob

package_name = 'external_connections'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Web files are now served from the standalone web_monitor directory
        # and are no longer part of this package
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='tamas',
    maintainer_email='user@todo.todo',
    description='Package for external connections to BORS robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = external_connections.web_server:main',
            'connection_manager = external_connections.connection_manager:main',
        ],
    },
)
