import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import pandas as pd


# Which sub-action CSVs (in gait_data/) make up the "forward" gait, per
# configuration. "backward" is generated automatically by reversing the
# flattened sequence. Add a new top-level key here once gait data exists
# for a new configuration (e.g. "hexapod").
GAIT_CONFIGS = {
    "turtle": {
        "forward": ["stand_stand", "stand_step", "stand_propel", "stand_prone"],
    },
    # "hexapod": {
    #     "forward": [...],
    # },
}

JOINT_NAMES = ["joint_0", "joint_1", "joint_2"]  # adjust to match real URDF joint names


def degrees_to_radians(row) -> list[float]:
    return [math.radians(v) for v in row]


class GaitManager(Node):
    def __init__(self):
        super().__init__("gait_manager")

        # -- Required parameters, supplied by Brain at launch --------------
        self.declare_parameter("worm_names", [""])
        self.declare_parameter("configuration", "")
        # Phase offset (in waypoint-table indices) for each worm, in the
        # same order as worm_names. Defaults to all-zero (no offset) if
        # not provided -- e.g. useful for a first test before real offsets
        # are worked out with the simulator.
        self.declare_parameter("phase_offsets", [0])

        worm_names = list(self.get_parameter("worm_names").value)
        self.configuration = self.get_parameter("configuration").value
        phase_offsets = list(self.get_parameter("phase_offsets").value)

        if not worm_names or worm_names == [""]:
            raise RuntimeError(
                "GaitManager requires a non-empty 'worm_names' parameter "
                "(the list of worm namespaces in this configuration)."
            )
        if not self.configuration:
            raise RuntimeError(
                "GaitManager requires a non-empty 'configuration' parameter "
                "(e.g. 'turtle', 'hexapod')."
            )
        if self.configuration not in GAIT_CONFIGS:
            raise RuntimeError(
                f"Unknown configuration '{self.configuration}'. "
                f"Known configurations: {list(GAIT_CONFIGS)}"
            )

        if phase_offsets == [0] and len(worm_names) != 1:
            # No real offsets provided -- default every worm to zero offset
            # rather than mis-cycling through a length-1 list.
            phase_offsets = [0] * len(worm_names)
        if len(phase_offsets) != len(worm_names):
            raise RuntimeError(
                f"'phase_offsets' length ({len(phase_offsets)}) must match "
                f"'worm_names' length ({len(worm_names)})."
            )

        self.worm_names = worm_names
        self.phase_offsets = dict(zip(worm_names, phase_offsets))

        self.declare_parameter("angular_speed_deg_s", 30.0)
        self.angular_speed_rad_s = math.radians(
            self.get_parameter("angular_speed_deg_s").value
        )

        self.declare_parameter("timer_period_s", 0.05)
        self.timer_period_s = self.get_parameter("timer_period_s").value

        self.declare_parameter("ready_tolerance_deg", 3.0)
        self.ready_tolerance_rad = math.radians(
            self.get_parameter("ready_tolerance_deg").value
        )

        self.script_directory = os.path.dirname(os.path.realpath(__file__))

        # Cache of fully-flattened, radian-converted waypoint sequences,
        # keyed by gait name ("forward", "backward", ...) within the
        # currently selected configuration.
        self._gait_cache: dict[str, list[list[float]]] = {}

        self.current_gait: str | None = None
        self.waypoints: list[list[float]] = []
        # Single shared clock: an index into `waypoints`. Each worm's own
        # target is waypoints[(base_waypoint_index + its offset) % len].
        self.base_waypoint_index = 0

        # Most recent measured joint state per worm.
        self.last_measured_position: dict[str, list[float]] = {}

        self.desired_state_publishers: dict[str, object] = {}
        for worm in self.worm_names:
            self.create_subscription(
                JointState,
                f"/{worm}/joint_states",
                self._make_joint_state_callback(worm),
                10,
            )
            self.desired_state_publishers[worm] = self.create_publisher(
                JointState, f"/{worm}/desired_joint_state", 10
            )

        self.cmd_subscriber = self.create_subscription(
            String, "cmd_gait", self.cmd_callback, 10
        )

        self.timer = self.create_timer(self.timer_period_s, self.timer_callback)

        self.get_logger().info(
            f"GaitManager started for configuration '{self.configuration}' "
            f"with worms {self.worm_names} (offsets {self.phase_offsets})"
        )

    # -- gait table loading ---------------------------------------------

    def _load_action_csv(self, action_name: str) -> list[list[float]]:
        path = os.path.join(self.script_directory, "gait_data", f"{action_name}.csv")
        df = pd.read_csv(path)
        return [degrees_to_radians(row) for row in df.values]

    def _build_gait(self, gait_name: str) -> list[list[float]]:
        config_gaits = GAIT_CONFIGS[self.configuration]

        if gait_name in config_gaits:
            waypoints: list[list[float]] = []
            for action_name in config_gaits[gait_name]:
                waypoints.extend(self._load_action_csv(action_name))
            return waypoints

        if gait_name == "backward" and "forward" in config_gaits:
            forward = self._get_gait_waypoints("forward")
            return list(reversed(forward))

        raise ValueError(
            f"Unknown gait '{gait_name}' for configuration '{self.configuration}'. "
            f"Known gaits: {list(config_gaits)}"
        )

    def _get_gait_waypoints(self, gait_name: str) -> list[list[float]]:
        if gait_name not in self._gait_cache:
            self._gait_cache[gait_name] = self._build_gait(gait_name)
        return self._gait_cache[gait_name]

    # -- ROS callbacks ----------------------------------------------------

    def cmd_callback(self, msg: String):
        gait_name = msg.data
        if gait_name == self.current_gait:
            return  # already doing this gait, nothing to do

        try:
            waypoints = self._get_gait_waypoints(gait_name)
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(f"Cannot start gait '{gait_name}': {e}")
            return

        self.current_gait = gait_name
        self.waypoints = waypoints
        self.base_waypoint_index = 0
        self.get_logger().info(
            f"Starting gait '{gait_name}' (configuration '{self.configuration}')"
        )

    def _make_joint_state_callback(self, worm: str):
        def callback(msg: JointState):
            self.last_measured_position[worm] = list(msg.position)
        return callback

    # -- main loop --------------------------------------------------------

    def _target_for(self, worm: str) -> list[float]:
        index = (self.base_waypoint_index + self.phase_offsets[worm]) % len(self.waypoints)
        return self.waypoints[index]

    def _worm_is_at_target(self, worm: str) -> bool:
        measured = self.last_measured_position.get(worm)
        if measured is None:
            return False
        target = self._target_for(worm)
        if len(measured) != len(target):
            return False
        return all(
            abs(actual - desired) <= self.ready_tolerance_rad
            for actual, desired in zip(measured, target)
        )

    def _all_worms_ready(self) -> bool:
        return all(self._worm_is_at_target(worm) for worm in self.worm_names)

    def timer_callback(self):
        if not self.waypoints:
            return
        if len(self.last_measured_position) < len(self.worm_names):
            return  # still waiting on initial feedback from every worm

        if self._all_worms_ready():
            # Every worm reached its own phase-shifted target -- advance
            # the shared clock so all worms move to their next waypoint.
            self.base_waypoint_index = (self.base_waypoint_index + 1) % len(self.waypoints)

        max_step = self.angular_speed_rad_s * self.timer_period_s

        for worm in self.worm_names:
            measured = self.last_measured_position[worm]
            target = self._target_for(worm)

            next_position = []
            for actual, desired in zip(measured, target):
                diff = desired - actual
                if abs(diff) <= max_step:
                    next_position.append(desired)
                else:
                    next_position.append(actual + math.copysign(max_step, diff))

            msg = JointState()
            msg.name = JOINT_NAMES
            msg.position = next_position
            self.desired_state_publishers[worm].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
