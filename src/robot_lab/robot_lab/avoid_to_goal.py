"""Drive to a fixed (x, y) while steering around obstacles seen by the lidar."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from robot_lab.geometry import (clamp, corridor_min, normalize_angle,
                                sector_min, yaw_from_quaternion)

SEEKING = 'seeking'
AVOIDING = 'avoiding'
ESCAPING = 'escaping'


class AvoidToGoal(Node):

    def __init__(self):
        super().__init__('avoid_to_goal')

        self.declare_parameter('goal_x', 2.0)           # meters, odom frame
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.15)

        self.declare_parameter('linear_speed', 0.20)    # ceiling, m/s
        self.declare_parameter('angular_speed', 1.00)   # ceiling, rad/s
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.5)

        self.declare_parameter('corridor_half_width', 0.16)

        self.declare_parameter('lookahead', 0.55)
        self.declare_parameter('clear_lookahead', 0.80)
        self.declare_parameter('emergency_distance', 0.25)  # rotate in place
        self.declare_parameter('avoid_linear_speed', 0.06)
 
        self.declare_parameter('escape_distance', 0.40)

        self.declare_parameter('min_side_clearance', 0.45)
        self.declare_parameter('side_sector_centre', 60.0)  # degrees
        self.declare_parameter('side_sector_width', 90.0)   # degrees

        self.declare_parameter('stamped', True)
        self.declare_parameter('frame_id', '')

        g = self.get_parameter
        self.goal = (g('goal_x').value, g('goal_y').value)
        self.goal_tolerance = g('goal_tolerance').value
        self.linear_speed = g('linear_speed').value
        self.angular_speed = g('angular_speed').value
        self.linear_gain = g('linear_gain').value
        self.angular_gain = g('angular_gain').value
        self.corridor_half_width = g('corridor_half_width').value
        self.lookahead = g('lookahead').value
        self.clear_lookahead = g('clear_lookahead').value
        self.emergency_distance = g('emergency_distance').value
        self.avoid_linear_speed = g('avoid_linear_speed').value
        self.escape_distance = g('escape_distance').value
        self.min_side_clearance = g('min_side_clearance').value
        self.side_centre = math.radians(g('side_sector_centre').value)
        self.side_half_width = math.radians(g('side_sector_width').value / 2.0)
        self.stamped = g('stamped').value
        self.frame_id = g('frame_id').value

        msg_type = TwistStamped if self.stamped else Twist
        self.publisher = self.create_publisher(msg_type, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_subscription(LaserScan, '/scan', self.scan_callback,
                                 qos_profile_sensor_data)

        self.x = self.y = self.yaw = None
        self.scan = None
        self.start = None
        self.last_xy = None
        self.path_length = 0.0
        self.arrived = False
        self.mode = SEEKING
        self.turn_sign = 1.0        # held for the duration of one avoid episode
        self.avoid_episodes = 0
        self.escape_start = None

        self.create_timer(0.05, self.tick)      # 20 Hz control
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'avoid_to_goal started: target ({self.goal[0]:.2f}, {self.goal[1]:.2f}), '
            f'corridor {2 * self.corridor_half_width:.2f} m wide, '
            f'lookahead {self.lookahead:.2f} m, publishing {msg_type.__name__}'
        )

    def send(self, linear, angular):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            twist = msg.twist
        else:
            msg = Twist()
            twist = msg
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(msg)

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        if self.last_xy is not None:
            self.path_length += math.dist(self.last_xy, (self.x, self.y))
        self.last_xy = (self.x, self.y)
        if self.start is None:
            self.start = (self.x, self.y)
            self.get_logger().info(
                f'first odometry: ({self.x:.2f}, {self.y:.2f}) -- '
                f'{math.dist(self.start, self.goal):.2f} m from the goal'
            )

    def errors(self):
        dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
        return math.hypot(dx, dy), normalize_angle(math.atan2(dy, dx) - self.yaw)

    def openness(self, centre):
        """How much room on one side. None (no return at all) is maximal room."""
        hit = sector_min(self.scan, centre, self.side_half_width)
        return float('inf') if hit is None else hit.range

    def choose_turn(self, heading_error):
        """Turn toward the goal if that side is passable, else toward room. """
        left = self.openness(self.side_centre)
        right = self.openness(-self.side_centre)
        goal_side = 1.0 if heading_error >= 0.0 else -1.0
        goal_side_room = left if goal_side > 0 else right
        if goal_side_room >= self.min_side_clearance:
            return goal_side, left, right
        return (1.0 if left > right else -1.0), left, right

    def start_avoiding(self, hazard, heading_error):
        self.mode = AVOIDING
        self.avoid_episodes += 1
        # Commit to one direction per episode
        self.turn_sign, left, right = self.choose_turn(heading_error)
        self.get_logger().info(
            f'AVOIDING: obstacle {hazard.range:.2f} m ahead at '
            f'{math.degrees(hazard.bearing):+.0f} deg; left {left:.2f} m, '
            f'right {right:.2f} m -> turning '
            f'{"left" if self.turn_sign > 0 else "right"}'
        )

    def tick(self):
        if self.x is None or self.scan is None or self.arrived:
            return

        distance, heading_error = self.errors()
        if distance <= self.goal_tolerance:
            self.arrive(distance)
            return

        horizon = self.clear_lookahead if self.mode == AVOIDING else self.lookahead
        hazard = corridor_min(self.scan, horizon, self.corridor_half_width)

        if hazard is not None and self.mode != AVOIDING:
            self.start_avoiding(hazard, heading_error)

        if self.mode == AVOIDING:
            if hazard is None:
                self.mode = ESCAPING
                self.escape_start = (self.x, self.y)
                self.get_logger().info(
                    f'ESCAPING: corridor clear, driving {self.escape_distance:.2f} m '
                    'straight before seeking again')
            else:
                linear = (0.0 if hazard.range <= self.emergency_distance
                          else self.avoid_linear_speed)
                self.send(linear, self.turn_sign * self.angular_speed)
                return

        if self.mode == ESCAPING:
            gone = math.dist(self.escape_start, (self.x, self.y))
            if gone < self.escape_distance:
                self.send(min(self.linear_speed, 0.15), 0.0)
                return
            self.mode = SEEKING
            self.get_logger().info(
                f'SEEKING: escaped {gone:.2f} m, {distance:.2f} m to go')

        angular = math.copysign(
            clamp(self.angular_gain * abs(heading_error), 0.0, self.angular_speed),
            heading_error)
        linear = (clamp(self.linear_gain * distance, 0.0, self.linear_speed)
                  * max(0.0, math.cos(heading_error)))
        self.send(linear, angular)

    def report(self):
        if self.x is None or self.arrived:
            return
        distance, heading_error = self.errors()
        self.get_logger().info(
            f'[{self.mode}] at ({self.x:+.2f}, {self.y:+.2f}) | {distance:.2f} m to go, '
            f'heading off by {math.degrees(heading_error):+.0f} deg'
        )

    def arrive(self, distance):
        self.arrived = True
        self.send(0.0, 0.0)
        direct = math.dist(self.start, (self.x, self.y))
        ratio = self.path_length / direct if direct > 1e-6 else float('nan')
        self.get_logger().info(
            f'arrived: ({self.x:+.3f}, {self.y:+.3f}), {distance:.3f} m from the goal '
            f'after {self.avoid_episodes} avoid episode(s). '
            f'Path {self.path_length:.2f} m vs {direct:.2f} m direct ({ratio:.2f}x).'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AvoidToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
