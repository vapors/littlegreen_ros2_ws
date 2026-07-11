from glob import glob
from setuptools import find_packages, setup

package_name = 'pd_controller_pkg'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/pd_controller_pkg/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scott',
    maintainer_email='scott@todo.todo',
    description='Safety envelope and configurable outer-loop position controller for LittleGreen.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pd_controller_node = pd_controller_pkg.pd_controller_node:main',
        ],
    },
)
