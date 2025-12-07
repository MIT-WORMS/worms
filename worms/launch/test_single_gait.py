from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="worms",
                # TODO: Accept this as a launch argument
                namespace="pony",
                executable="cubemars_interface",
                name="cubemars_interface",
                parameters=[
                    {
                        "can_channel": "can0",
                        "joint_state_freq": 10.0,
                        # TODO: Determine mapping
                        # TODO: Look these up from a config file
                        # "joint_nums": [0, 1, 2],
                        # "can_ids": [0x00, 0x00, 0x00],
                    }
                ],
            )
        ]
    )
