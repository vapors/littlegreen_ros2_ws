from setuptools import setup

package_name = 'joystick_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scott',
    maintainer_email='scott@example.com',
    description='Bridges /cmd_vel to /tmp/joystick_cmd.txt for Isaac Sim joystick control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_file = joystick_bridge.cmd_vel_to_file:main',
        ],
    },
)
