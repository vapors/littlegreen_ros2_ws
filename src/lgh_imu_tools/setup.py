from glob import glob

from setuptools import find_packages, setup

package_name = "lgh_imu_tools"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Scott Milich",
    maintainer_email="scott.milich@me.com",
    description=(
        "Source-independent IMU preflight, characterization, orientation audit, "
        "and recording tools for LittleGreen."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "imu_preflight = lgh_imu_tools.imu_preflight:main",
            "stationary_characterization = lgh_imu_tools.stationary_characterization:main",
            "orientation_audit = lgh_imu_tools.orientation_audit:main",
            "imu_recorder = lgh_imu_tools.imu_recorder:main",
        ]
    },
)
