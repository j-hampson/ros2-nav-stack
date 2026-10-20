"""Log what the lidar sees without moving the robot."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from robot_lab.geometry import sector_min


SECTORS = (('front', 0.0), ('left', 90.0), ('rear', 180.0), ('right', -90.0))


class ScanInspector(Node):

    def __init__(self):
        super().__init__('scan_inspector')

        self.declare_parameter('sector_width', 90.0)   # degrees, full width
        self.declare_parameter('report_period', 0.5)   # seconds

        self.half_width = math.radians(self.get_parameter('sector_width').value / 2.0)
        period = self.get_parameter('report_period').value

        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.scan = None
        self.described = False
        self.timer = self.create_timer(period, self.report)
        self.get_logger().info('scan_inspector started, waiting for /scan')

    def scan_callback(self, msg):
        if not self.described:
            self.described = True
            self.describe(msg)
        self.scan = msg

    def describe(self, msg):
        """One-off dump of the sensor's geometry."""
        n = len(msg.ranges)
        finite = sum(1 for r in msg.ranges if math.isfinite(r))
        self.get_logger().info(
            f'lidar: {n} rays over '
            f'[{math.degrees(msg.angle_min):.1f}, {math.degrees(msg.angle_max):.1f}] deg, '
            f'{math.degrees(msg.angle_increment):.2f} deg apart; '
            f'range [{msg.range_min:.2f}, {msg.range_max:.2f}] m; '
            f'frame {msg.header.frame_id!r}'
        )
        self.get_logger().info(
            f'ranges[0] points at {math.degrees(msg.angle_min):+.1f} deg '
            f'(0 = straight ahead); {finite}/{n} rays currently return a value'
        )

    def report(self):
        if self.scan is None:
            self.get_logger().warn('no /scan yet -- is the sim running?',
                                   throttle_duration_sec=5.0)
            return

        parts = []
        for label, centre_deg in SECTORS:
            hit = sector_min(self.scan, math.radians(centre_deg), self.half_width)
            if hit is None:
                parts.append(f'{label}: --- ')
            else:
                parts.append(f'{label}: {hit.range:4.2f}m@{math.degrees(hit.bearing):+5.0f}')

        nearest = sector_min(self.scan, 0.0, math.pi)  
        overall = ('nothing in range' if nearest is None else
                   f'{nearest.range:.2f}m at {math.degrees(nearest.bearing):+.0f} deg')
        self.get_logger().info('  '.join(parts) + f'   | nearest overall: {overall}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanInspector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
