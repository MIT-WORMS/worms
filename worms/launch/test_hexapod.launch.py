import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Existing launch file for the brain + raft
    worms_cortex_dir = get_package_share_directory("worms_cortex")
    brain_launch = os.path.join(worms_cortex_dir, "launch", "brain.launch.py")

    # Assets paths for topology
    worms_assets_dir = get_package_share_directory("worms_assets")
    macros_dir = os.path.join(worms_assets_dir, "xacro", "macros")
    urdf_dir = os.path.join(worms_assets_dir, "urdf")

    def make_worm(namespace: str, module_id: int, chassis_port: str) -> tuple:
        """Launches all startup nodes for a single worm in the hexapod body"""
        return (
            # Launches this worm's brain and raft instance
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(brain_launch),
                launch_arguments={
                    "namespace": namespace,
                    "node_id": str(module_id),
                    "cluster_name": "raft_cluster",
                    "foros_inspector": "1",
                }.items(),
            ),
            Node(
                package="worms",
                executable="topology_manager",
                name="topology_manager",
                namespace=namespace,
                parameters=[
                    {
                        "topology.xacro_dir": macros_dir,
                        "topology.urdf_dir": urdf_dir,
                        "module.module_id": module_id,
                        "base.remote_module_id": 12,  # Chassis ID
                        "base.remote_module_class": "hexapod_chassis",
                        "base.remote_port_name": chassis_port,
                        "end.remote_module_id": 20 + module_id,
                        "end.remote_module_class": "rubber_shoe",
                        "end.remote_port_name": "center",
                    }
                ],
                output="screen",
            ),
        )

    return LaunchDescription(
        [
            *make_worm("pony", 1, "back_right"),
            *make_worm("frog", 2, "front_right"),
            *make_worm("lion", 3, "back_left"),
            *make_worm("duck", 4, "front_left"),
            *make_worm("seal", 5, "middle_left"),
            *make_worm("hare", 6, "middle_right"),
        ]
    )
