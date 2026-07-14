from glob import glob
from setuptools import find_packages, setup

package_name = 'lgh_st3215_tools'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Scott Milich',
    maintainer_email='scott.milich@me.com',
    description='Guarded calibration, characterization, preflight, auditing, and dataset tools for the LGH ST3215 system.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_console = lgh_st3215_tools.pose_console:main',
            'assume_policy_default = lgh_st3215_tools.pose_console:main',
            'print_default_pose = lgh_st3215_tools.print_default_pose:main',
            'print_policy_default = lgh_st3215_tools.print_default_pose:main',
            'print_model_zero = lgh_st3215_tools.print_default_pose:main_model_zero',
            'capture_calibration = lgh_st3215_tools.capture_calibration:main',
            'apply_calibration = lgh_st3215_tools.apply_calibration:main',
            'verify_calibration = lgh_st3215_tools.verify_reference_pose:main_model_zero',
            'verify_model_zero = lgh_st3215_tools.verify_reference_pose:main_model_zero',
            'verify_policy_default = lgh_st3215_tools.verify_reference_pose:main_policy_default',
            'servo_identification = lgh_st3215_tools.servo_identification:main',
            'standing_characterization = lgh_st3215_tools.standing_characterization:main',
            'st3215_preflight = lgh_st3215_tools.st3215_preflight:main',
            'hardware_snapshot = lgh_st3215_tools.hardware_snapshot:main',
            'dataset_manifest = lgh_st3215_tools.dataset_manifest:main',
        ],
    },
)
