import rclpy
from rclpy.node import Node

from worms_topology_monitor.monitor import TopologyMonitor, ModuleMonitor


class TopologyManager(Node):
    """
    Tracks system topology and simultaneously monitors the module it is run on. Must
    set `topology.xacro_dir` to allow urdf compilation and `module.module_id` to
    indicate what module is being tracked. The tracked module must be registered in
    the hardware database as a module with the Monitored capability.
    """

    def __init__(self):
        super().__init__("topology_manager")

        # Topology tracking parameters
        self.declare_parameter("topology.stale_timeout_s", 3.0)
        self.declare_parameter("topology.xacro_dir", rclpy.Parameter.Type.STRING)
        self.declare_parameter("topology.urdf_dir", rclpy.Parameter.Type.STRING)

        self.topology_monitor = TopologyMonitor(
            self,
            timeout_s=self.get_parameter("topology.stale_timeout_s")
            .get_parameter_value()
            .double_value,
            xacro_dir=self.get_parameter("topology.xacro_dir")
            .get_parameter_value()
            .string_value,
            urdf_dir=self.get_parameter("topology.urdf_dir")
            .get_parameter_value()
            .string_value,
        )

        # Module tracking parameters
        self.declare_parameter("module.module_id", rclpy.Parameter.Type.INTEGER)
        self.declare_parameter("module.hb_freq", 10.0)

        self.module_monitor = ModuleMonitor(
            self,
            module_id=self.get_parameter("module.module_id")
            .get_parameter_value()
            .integer_value,
            hb_freq=self.get_parameter("module.hb_freq")
            .get_parameter_value()
            .double_value,
        )


def main(args=None):
    rclpy.init(args=args)
    node = TopologyManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
