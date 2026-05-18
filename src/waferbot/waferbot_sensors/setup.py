from setuptools import find_packages, setup
from glob import glob

package_name = "waferbot_sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        # (destination folder, list of files to copy),
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob(f"launch/*_launch.py")),
        (f"share/{package_name}/config", glob(f"config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="daniel",
    maintainer_email="daniel.onderk@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"imu_node = {package_name}.imu_node:main",
            f"camera_node = {package_name}.camera_node:main",
            f"servo_node = {package_name}.servo_node:main",
            f"ultrasonic_node = {package_name}.ultrasonic_node:main",
        ],
    },
)
