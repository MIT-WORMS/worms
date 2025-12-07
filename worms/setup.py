from glob import glob
from setuptools import find_packages, setup

package_name = "worms"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "cubemars-pycan @ git+https://github.com/MIT-WORMS/cubemars-pycan.git",
    ],
    zip_safe=True,
    maintainer="trevor",
    maintainer_email="worms@mit.edu",
    description="Hardware specific code for the WORMS robot.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cubemars_interface = worms.worms.hardware.cubemars_interface:main",
        ],
    },
)
