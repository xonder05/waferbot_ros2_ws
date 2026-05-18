from setuptools import find_packages, setup
from glob import glob

package_name = "waferbot_gazebo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        # (destination folder, list of files to copy),
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*_launch.py")),
        (f"share/{package_name}/launch/helpers", glob("launch/helpers/*_launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/worlds/garden", glob("worlds/garden/*.sdf")),
        (f"share/{package_name}/worlds/wandering", glob("worlds/wandering/*.sdf")),
        (f"share/{package_name}/worlds/warehouse", glob("worlds/warehouse/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="daniel",
    maintainer_email="daniel.onderk@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            f"ultrasonic_interpreter = {package_name}.ultrasonic_interpreter:main",
            f"image_compressor = {package_name}.image_compressor:main",
        ],
    },
)
