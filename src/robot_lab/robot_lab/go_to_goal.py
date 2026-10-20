"""Drive to a fixed (x, y) in the odometry frame."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry

from robot_lab.geometry import clamp, normalize_angle, yaw_from_quaternion


class GoToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal')

        self.declare_parameter('goal_x', 1.5)           # meters, odom frame
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.10)  # meters

        self.declare_parameter('linear_speed', 0.20)    # ceiling, m/s
        self.declare_parameter('angular_speed', 1.00)   # ceiling, rad/s
        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.5)

        self.declare_parameter('stamped', True)         # TwistStamped vs Twist
        self.declare_parameter('frame_id', '')

        self.goal = (self.get_parameter('goal_x').value,
                     self.get_parameter('goal_y').value)
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.stamped = self.get_parameter('stamped').value
        self.frame_id = self.get_parameter('frame_id').value

        msg_type = TwistStamped if self.stamped else Twist
        self.publisher = self.create_publisher(msg_type, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.x = self.y = self.yaw = None
        self.start = None
        self.arrived = False
        self.path_length = 0.0  
        self.last_xy = None

        self.timer = self.create_timer(0.05, self.tick)   # 20 Hz
        self.report_timer = self.create_timer(1.0, self.report)
        self.get_logger().info(
            f'go_to_goal started: target ({self.goal[0]:.2f}, {self.goal[1]:.2f}) '
            f'in the odom frame, tolerance {self.goal_tolerance:.2f} m, '
            f'publishing {msg_type.__name__}'
        )

    def send(self, linear, angular):
        """Publish a velocity command in whichever type the robot expects."""
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

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.x, self.y = position.x, position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        # Sum the distance between consecutive samples
        if self.last_xy is not None:
            self.path_length += math.dist(self.last_xy, (self.x, self.y))
        self.last_xy = (self.x, self.y)
        if self.start is None:
            self.start = (self.x, self.y)
            self.get_logger().info(
                f'first odometry: ({self.x:.2f}, {self.y:.2f}) yaw {self.yaw:+.2f} '
                f'-- {math.dist(self.start, self.goal):.2f} m from the goal'
            )

    def errors(self):
        """Distance to the goal, and how far the nose is off the bearing to it."""
        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        distance = math.hypot(dx, dy)
        heading_error = normalize_angle(math.atan2(dy, dx) - self.yaw)
        return distance, heading_error

    def tick(self):
        if self.x is None or self.arrived:
            return

        distance, heading_error = self.errors()

        if distance <= self.goal_tolerance:
            self.arrive(distance)
            return

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
            f'at ({self.x:+.2f}, {self.y:+.2f}) yaw {self.yaw:+.2f} | '
            f'{distance:.2f} m to go, heading off by {math.degrees(heading_error):+.0f} deg'
        )

    def arrive(self, distance):
        self.arrived = True
        self.send(0.0, 0.0)
        self.timer.cancel()
        self.report_timer.cancel()
        
        # Compare the integrated path against the straight line to where it actually stopped
        direct = math.dist(self.start, (self.x, self.y))
        ratio = self.path_length / direct if direct > 1e-6 else float('nan')
        self.get_logger().info(
            f'arrived: ({self.x:+.3f}, {self.y:+.3f}), {distance:.3f} m from the goal. '
            f'Path {self.path_length:.2f} m vs {direct:.2f} m direct ({ratio:.2f}x).'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.send(0.0, 0.0)   # do not leave the robot driving
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
