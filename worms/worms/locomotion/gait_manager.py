import math
from collections import defaultdict
from collections.abc import Callable

import rclpy
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.subscription import Subscription
from sensor_msgs.msg import JointState
from worms_arthron.hardware import Module
from worms_arthron.hardware.capability import Actuated
from worms_arthron.infra import get_database
from worms_arthron_msgs.msg import NamedConfiguration
from worms_cortex.subroutine_node import SubroutineNode
from worms_mission_control.command_registry import (
    CommandArgument,
    CommandDescription,
    CommandRegistry,
)

REGISTRY = CommandRegistry()
REGISTRY.register(
    CommandDescription(
        name="walk",
        package="worms",
        executable="gait_manager",
        node_name="gait_manager",
        compatible_configs=["turtle", "hexapod", "quadruped"],
        description="Walk forwards with a fixed gait.",
        group="locomotion",
        args=[
            CommandArgument(
                name="direction",
                type=str,
                description="Which direction to walk in.",
                required=True,
                choices=["forward", "backward"],
            )
        ],
    )
)


class GaitManager(SubroutineNode):
    def __init__(self):
        super().__init__(
            "gait_manager",
            registry=REGISTRY,
            automatically_declare_parameters_from_overrides=True,  # type: ignore
        )
        self._db = get_database()

        # Parameters (cant declare anything with the automatically_... flag above)
        self.angular_speed_rad_s = math.radians(
            self.get_parameter("angular_speed_deg_s").get_parameter_value().double_value
        )
        self.joint_tol_rad = math.radians(
            self.get_parameter("joint_tol_deg").get_parameter_value().double_value
        )

        # Gait is loaded dynamically from config
        self.gait = self._get_gait_parameter()
        self.gait_len = len(next(iter(self.gait.values())))

        # Get the current configuration on startup
        self._config: dict[str, Module]
        self._config_name = ""
        self.create_subscription(
            NamedConfiguration,
            "named_configuration",
            self._configuration_callback,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

        # Do not start the node until we have a configuration
        while rclpy.ok() and not self._config_name:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self._config_name:
            return

        # Parse the configuration for actuated modules and verify gait
        self.actuated: dict[str, Actuated] = {
            eid: m for eid, m in self._config.items() if isinstance(m, Actuated)
        }

        missing = [eid for eid in self.actuated if eid not in self.gait]
        if missing:
            self.get_logger().fatal(f"Missing gait entries for: {missing}")
            rclpy.try_shutdown()

        extra = [eid for eid in self.gait if eid not in self.actuated]
        if extra:
            self.get_logger().fatal(f"Unrecognized gait entries: {extra}")
            rclpy.try_shutdown()

        # Persistent state
        self.cur_joint_pos: dict[str, list[float]] = {}
        self.prev_joint_pos: dict[str, list[float]] = {}
        self.waypoint_num = 0
        self.waypoint_increment = 0

        # Setup each actuated module
        self.subs: list[Subscription] = []
        self.pubs: dict[str, Publisher] = {}
        self.joints: dict[str, list[str]] = {}
        for eid, mod in self.actuated.items():
            assert isinstance(mod, Module)
            ns = mod.get_namespace()

            # Subscribe to the module's state
            self.subs.append(
                self.create_subscription(
                    JointState,
                    f"/{ns}/joint_states",
                    self._make_joint_state_callback(eid),
                    10,
                )
            )

            # Publish to the module's command
            self.pubs[eid] = self.create_publisher(
                JointState,
                f"/{ns}/joint_commands",
                10,
            )

            # Store this module's joint names
            self.joints[eid] = [
                joint_name.format(namespace=ns) for joint_name in mod.joint_names
            ]

        # Primary gait loop
        self.gait_period_s = 0.1
        self.create_timer(self.gait_period_s, self._gait_callback)

    def on_command(self, command: str, args: dict) -> None:
        """Process a new command for the gait manager and update logic."""
        match command:
            case "walk":
                direction = args["direction"]
                self.waypoint_increment = 1 if direction == "forward" else -1
            case _:
                self.get_logger().error(f"Unknown command: {command}")

    def _gait_callback(self) -> None:
        """Periodic callback for advancing the gait when needed."""
        all_ready = all(
            self._is_at_target(eid, self.gait[eid][self.waypoint_num])
            for eid in self.actuated
        )

        # Advance to next waypoint if all joints have reached the target
        if all_ready:
            self.waypoint_num += self.waypoint_increment
            self.waypoint_num %= self.gait_len

        max_step = self.angular_speed_rad_s * self.gait_period_s
        stamp = self.get_clock().now().to_msg()

        for eid, waypoints in self.gait.items():
            measured = self.cur_joint_pos.get(eid)
            if measured is None:
                continue

            target = waypoints[self.waypoint_num]

            # Start from measured on first pass then open-loop follow gait
            if self.prev_joint_pos.get(eid) is None:
                self.prev_joint_pos[eid] = list(measured)
            reference = self.prev_joint_pos[eid]

            # Interpolate towards the next target
            cmd_position = []
            for ref, des in zip(reference, target):
                diff = des - ref
                if abs(diff) <= max_step:
                    cmd_position.append(des)
                else:
                    cmd_position.append(ref + math.copysign(max_step, diff))
            self.prev_joint_pos[eid] = cmd_position

            # Publish this new joint command
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = self.joints[eid]
            msg.position = cmd_position
            self.pubs[eid].publish(msg)

    def _is_at_target(self, entry_id: str, target: list[float]) -> bool:
        """
        Determines if the selected module entry is at an indicated target position.

        Args:
            entry_id (str): The name of the module entry to check
            target ([float]): The target position to compare against

        Returns:
            at_target (bool): True if all joints match the target position within tol
        """
        measured = self.cur_joint_pos.get(entry_id)
        if measured is None:
            return False

        return all(
            abs(actual - desired) <= self.joint_tol_rad
            for actual, desired in zip(measured, target)
        )

    def _get_gait_parameter(self) -> dict[str, list[list[float]]]:
        """
        Dynamically read the gait sequence from the parameters.

        Returns:
            gait ({str: [[float]]}): A mapping from each actuated config entry to
                a sequence of its joint angles for each waypoint.
        """
        raw: dict[str, dict[str, list[float]]] = defaultdict(dict)

        for name in self._parameters:
            # Not a gait entry
            if not name.startswith("gait."):
                continue

            # Waypoints are stored like 'gait.cfg_id.n'
            _, cfg_id, waypoint_n = name.split(".", maxsplit=2)
            raw[cfg_id][waypoint_n] = list(
                self.get_parameter(name).get_parameter_value().double_array_value
            )

        # Verify and process all waypoints
        gait: dict[str, list[list[float]]] = {}
        num_waypoints = None
        for cfg_id, waypoints in raw.items():
            # Verify the length of the waypoints
            if num_waypoints is None:
                num_waypoints = len(waypoints.values())
            elif len(waypoints.values()) != num_waypoints:
                self.get_logger().fatal(
                    f"{cfg_id} has {len(waypoints.values())}, expected {num_waypoints}"
                )
                rclpy.try_shutdown()

            # Sort waypoints assuming the waypoint key is just a number
            gait[cfg_id] = [
                [math.radians(angle) for angle in waypoint]
                for _, waypoint in sorted(
                    waypoints.items(), key=lambda item: int(item[0])
                )
            ]

        # Accessed like gait[cfg_id][waypoint_num][joint_num]
        return gait

    def _configuration_callback(self, msg: NamedConfiguration) -> None:
        """
        Subscription callback for the named system configuration. Stores the config
        on first observation and errors if it is later changed.
        """
        if self._config_name and self._config_name != msg._config_name:
            self.get_logger().fatal(
                "Configuration unexpectedly changed during operation, "
                "terminating subroutine."
            )
            rclpy.try_shutdown()
            return

        self._config_name = msg.config_name
        self._config = {
            entry.entry_id: self._db.instance_registry[entry.module_id]
            for entry in msg.entries
        }

    def _make_joint_state_callback(self, entry_id: str) -> Callable[[JointState], None]:
        """
        Makes and returns a callback that records for a joint state subscriber.

        Args:
            entry_id (str): The name of the module entry to track

        Returns:
            callback (Callable): A ROS subscriber callback for this entry
        """

        def callback(msg: JointState) -> None:
            self.cur_joint_pos[entry_id] = list(msg.position)

        return callback


def main(args=None):
    rclpy.init(args=args)
    node = GaitManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
