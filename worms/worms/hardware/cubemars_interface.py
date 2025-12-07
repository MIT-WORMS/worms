import numpy as np
from pprint import pformat

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from cubemars_pycan.interface import CubeMarsCAN


class CubeMarsInterfaceNode(Node):
    def __init__(self):
        super().__init__("cubemars_interface")

        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("joint_state_freq", 10.0)
        self.declare_parameter("joint_nums")
        self.declare_parameter("can_ids")

        can_channel = (
            self.get_parameter("can_channel").get_parameter_value().string_value
        )
        joint_state_freq = (
            self.get_parameter("joint_state_freq").get_parameter_value().double_value
        )
        joint_nums = (
            self.get_parameter("joint_nums").get_parameter_value().integer_array_value
        )
        can_ids = (
            self.get_parameter("can_ids").get_parameter_value().integer_array_value
        )

        # Initialize the interface
        self._interface = CubeMarsCAN(can_channel)
        self._can_ids = {
            joint_num: can_id for joint_num, can_id in zip(joint_nums, can_ids)
        }

        # Subscribers
        self._command_sub = self.create_subscription(
            JointTrajectory, "joint_state_desired", self._handle_joint_command, 10
        )

        # Publishers
        self._state_pub = self.create_publisher(JointState, "joint_state", 10)
        self.create_timer(1 / joint_state_freq, self._publish_joint_states)

        # Services
        self.create_service(Empty, "set_origin", self._set_origin_callback)

        self._prev_error_stamp = self.get_clock().now().nanoseconds / 1e9
        self._throttle_duration_sec = 1.0
        self.get_logger().info("CubeMars Interface has been initialized.")
        self.get_logger().debug(f"Parameters:\n{pformat(self._parameters)}")

    def _handle_joint_command(self, msg: JointTrajectory):
        """Handle an incoming joint command message."""
        if len(msg.points) != len(self._can_ids):
            self.get_logger().warn(
                f"Received JointTrajectory message with {len(msg.points)} joints, "
                f"but expected {len(self._can_ids)}."
            )
            return

        # Set each position to the motor
        for joint_num, point in enumerate(msg.points):
            position = point.positions[0]
            can_id = self._can_ids[joint_num]
            self._interface.set_position(can_id, np.rad2deg(position))

    def _publish_joint_states(self):
        """Publish the most recent joint state from the motors."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # TODO (trejohst): frame_id?

        names = []
        positions = []

        # Populate fields for each joint (and log errors)
        for joint_num, can_id in self._can_ids.items():
            maybe_pos = self._interface.pos_deg(can_id)
            maybe_err = self._interface.errors(can_id)
            if maybe_pos is None or maybe_err is None:
                self.get_logger().warn(
                    f"Could not read joint {joint_num} (CAN ID {can_id})",
                    throttle_duration_sec=self._throttle_duration_sec,
                )
                continue

            names.append(f"joint_{joint_num}")
            positions.append(np.deg2rad(maybe_pos))

            # TODO (trejohst): This is in ERPM, convert to rad/s
            # msg.velocity = self._interface.speed_rpm(can_id)
            # TODO (trejohst): This is current in amps, convert to Nm
            # msg.effort = self._interface.current_amp(can_id)

            # Log any new errors
            for fault, stamps in maybe_err.items():
                for stamp in stamps:
                    # We already logged this error
                    if stamp < self._prev_error_stamp:
                        continue

                    self.get_logger().error(
                        f"Motor fault on joint {joint_num} (CAN ID {can_id}): {fault.name}"
                    )

        self._prev_error_stamp = self.get_clock().now().nanoseconds / 1e9

        # Warn on unexpected motors
        recognized_can_ids = set(self._interface.can_ids())
        unexpected_can_ids = recognized_can_ids - set(self._can_ids.values())
        if len(unexpected_can_ids) > 0:
            self.get_logger().warn(
                f"Unexpected motors on CAN IDs: {unexpected_can_ids}",
                throttle_duration_sec=self._throttle_duration_sec,
            )

        # Populate and publish
        msg.name = names
        msg.position = positions
        self._state_pub.publish(msg)

    def _set_origin_callback(self, request: Empty.Request, response: Empty.Response):
        """Set the current position of each joint to the origin."""
        for can_id in self._can_ids.values():
            self._interface.set_origin(can_id)

        self.get_logger().info("Set current joint positions to origin.")

        return response


def main(args=None):
    rclpy.init(args=args)

    cubemars_interface = CubeMarsInterfaceNode()
    rclpy.spin(cubemars_interface)

    cubemars_interface.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
