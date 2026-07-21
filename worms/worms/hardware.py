from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from worms.topology.uib import MockUIB

from worms_topology_monitor.monitor.capability import Monitored

import worms_arthron.hardware.capability as cap
from worms_arthron.hardware import Module, XacroSpec, PortSpec
from worms_arthron.infra import register_module, InstanceRegistry, ConfigurationRegistry


"""
Worms
"""


@register_module()
class Worm(Module, Monitored, cap.Intelligent, cap.Actuated, cap.SelfSufficient):
    """The primary articulated linkage in the WORMS system."""

    module_class = "worm"
    port_specs = [
        PortSpec(
            port_name="base",
            attachment_link="parent_link",
            is_downstream=False,
        ),
        PortSpec(
            port_name="end",
            attachment_link="{namespace}_end_uib",
            is_downstream=True,
        ),
    ]
    xacro = XacroSpec(
        macro_file="3dof-worm.macro.xacro",
        macro_name="worm",
        namespaced=True,
    )

    # Specs from Actuated
    joint_names = [
        "{namespace}_joint_0",
        "{namespace}_joint_1",
        "{namespace}_joint_2",
    ]

    # Specs from Monitored
    port_monitors = {
        "base": MockUIB,
        "end": MockUIB,
    }


"""
Chassis
"""


@register_module()
class HexapodChassis(Module, cap.Static, cap.Unpowered):
    """A hexagonal chassis with connection ports for six UIBs."""

    module_class = "hexapod_chassis"
    port_specs = [
        PortSpec(
            port_name="front_left",
            attachment_link="front_left_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="front_right",
            attachment_link="front_right_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="middle_left",
            attachment_link="middle_left_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="middle_right",
            attachment_link="middle_right_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="back_left",
            attachment_link="back_left_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="back_right",
            attachment_link="back_right_uib",
            is_downstream=True,
        ),
    ]
    xacro = XacroSpec(
        macro_file="pallets.macro.xacro",
        macro_name="hexapod",
        namespaced=False,
    )


@register_module()
class TurtleChassis(Module, cap.Static, cap.Unpowered):
    """A rectangular chassis with four leg UIBs and two end UIBs."""

    module_class = "turtle_chassis"
    port_specs = [
        PortSpec(
            port_name="front_left",
            attachment_link="front_left_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="front_right",
            attachment_link="front_right_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="back_left",
            attachment_link="back_left_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="back_right",
            attachment_link="back_right_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="front_center",
            attachment_link="front_center_uib",
            is_downstream=True,
        ),
        PortSpec(
            port_name="back_center",
            attachment_link="back_center_uib",
            is_downstream=True,
        ),
    ]
    xacro = XacroSpec(
        macro_file="pallets.macro.xacro",
        macro_name="turtle",
        namespaced=False,
    )


"""
Species Modules
"""


@register_module()
class RubberShoe(Module, cap.Static, cap.Unpowered):
    """A simple flat rubber shoe with a single UIB."""

    module_class = "rubber_shoe"
    port_specs = [
        PortSpec(
            port_name="center",
            attachment_link="parent_link",
            is_downstream=False,
        )
    ]
    xacro = XacroSpec(
        macro_file="species.macro.xacro",
        macro_name="rubber_foot",
        namespaced=True,
    )


"""
Database registration
"""

config_dir = Path(get_package_share_directory("worms")) / "config"

INS_REGISTRY = InstanceRegistry.from_yaml(config_dir / "hardware.yaml")
CFG_REGISTRY = ConfigurationRegistry.from_directory(config_dir)
