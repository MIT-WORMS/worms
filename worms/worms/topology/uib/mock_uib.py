from worms_topology_monitor.monitor import PortMonitor

from std_srvs.srv import SetBool


class MockUIB(PortMonitor):
    """
    Basic UIB standin that advertises a connection status on update events. Update
    the remote connection by changing the node's parameters with:

    .. code-block:: bash

        ros2 param set /<node_name> <port_name>.remote_module_class turtle_chassis
        ros2 param set /<node_name> <port_name>.remote_module_id 3
        ros2 param set /<node_name> <port_name>.remote_port_name front_left

    The UIB should be disconnected before changing params, and reconnected after with:

    .. code-block:: bash

        ros2 service call /<namespace>/<node_name>/<port_name>/set_connected \
            std_srvs/srv/SetBool "{data: false}"
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        # Remote configuration
        self.node.declare_parameter(f"{self.port_name}.remote_port_name", "")
        self.node.declare_parameter(f"{self.port_name}.remote_module_class", "")
        self.node.declare_parameter(f"{self.port_name}.remote_module_id", 0)

        # Service to connect/disconnect
        self.srv = self.node.create_service(
            SetBool, f"~/{self.port_name}/set_connected", self.set_connected
        )

    def set_connection_handler(self, *args, **kwargs):
        """Extend to also emit a connection event on startup."""
        super().set_connection_handler(*args, **kwargs)
        self._handle_connection(True)

    def set_connected(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """Basic callback to toggle the connected state."""
        connected = request.data
        self._handle_connection(connected)
        response.success = True
        return response

    def _handle_connection(self, connected: bool) -> None:
        """Helper to handle a connect/disconnect event."""

        if connected:
            self.node.get_logger().info(f"{self.port_name} connected")
            # Advertise using the current parameter values
            self.advertise_connection(
                remote_port_name=self.node.get_parameter(
                    f"{self.port_name}.remote_port_name"
                )
                .get_parameter_value()
                .string_value,
                remote_module_class=self.node.get_parameter(
                    f"{self.port_name}.remote_module_class"
                )
                .get_parameter_value()
                .string_value,
                remote_module_id=self.node.get_parameter(
                    f"{self.port_name}.remote_module_id"
                )
                .get_parameter_value()
                .integer_value,
            )
        else:
            self.node.get_logger().info(f"{self.port_name} disconnected")
            self.advertise_disconnection()
