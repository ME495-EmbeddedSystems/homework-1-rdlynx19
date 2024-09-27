import rclpy
from rclpy.node import Node


class Waypoint(Node):
    """The waypoint Node"""

    def __init__(self):
        super().__init__('waypoint')
        self.get_logger().info('way_node')

        self.declare_parameter('frequency',90)
        self.freq = self.get_parameter('frequency').value
        
        self.timer = self.create_timer((1/self.freq),self.timer_callback)

    def timer_callback(self):
        self.get_logger().debug("Issuing Command!")


def main(args=None):
    rclpy.init(args=args)

    waypoint = Waypoint()

    rclpy.spin(waypoint)

    waypoint.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()