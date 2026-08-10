from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

package_name = "worms"


def package_files(root_str: str, package_name: str):
    root = Path(root_str)
    return [
        (
            str(Path("share") / package_name / path.parent),
            [str(path)],
        )
        for path in root.rglob("*")
        if path.is_file()
    ]


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*")),
        *package_files("config", package_name),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="worms",
    maintainer_email="worms@mit.edu",
    description="A multi-agent reconfigurable robot for space exploration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gait_manager = worms.locomotion.gait_manager:main",
            "topology_manager = worms.topology.topology_manager:main",
        ],
        "mission_control.subroutine_registries": [
            "gait_manager_registry = worms.locomotion.gait_manager:REGISTRY"
        ],
        "arthron.hardware_database": [
            "ins_registry = worms.hardware:INS_REGISTRY",
            "cfg_registry = worms.hardware:CFG_REGISTRY",
        ],
    },
)
