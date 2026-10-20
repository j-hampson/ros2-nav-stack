"""Drive a diff-drive robot around a square using closed-loop control.

Topic names are generic so the node works with any diff-drive robot; remap them
at launch. For the ros_gz_sim_demos diff_drive world:

    ros2 run robot_lab drive_square --ros-args \
      -r /cmd_vel:=/model/vehicle_blue/cmd_vel \
      -r /odom:=/model/vehicle_blue/odometry

Do not pass use_sim_time:=true unless /clock is bridged from Gazebo -- ROS time
would stay frozen at zero and the control timer would never fire.
"""

import math
from collections import namedtuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from robot_lab.geometry import clamp, normalize_angle, yaw_from_quaternion

DRIVING = 'driving'
TURNING = 'turning'

Pose2D = namedtuple('Pose2D', 'x y yaw')


class DriveSquare(Node):

    def __init__(self):
        super().__init__('drive_square')

        self.declare_parameter('side_length', 2.0)      # meters
        self.declare_parameter('linear_speed', 0.5)     # ceiling, m/s
        self.declare_parameter('angular_speed', 0.5)    # ceiling, rad/s
        self.declare_parameter('laps', 1)

        self.declare_parameter('linear_gain', 1.0)
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('min_angular_speed', 0.10)

        self.declare_parameter('distance_tolerance', 0.05)   # meters
        self.declare_parameter('angle_tolerance', 0.05)      # rad, ~2.9 deg

        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.laps = self.get_parameter('laps').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.pose = None            # no measurement until the first Odometry lands
        self.home = None            # where the square started, for the final report
        self.start_yaw = None       # heading all corner targets are measured from
        self.segment_start = None   # position the current side began at
        self.target_yaw = None      # heading the current turn is aiming for
        self.state = DRIVING
        self.sides_done = 0

        self.timer = self.create_timer(0.05, self.tick)  # 20 Hz control loop
        self.get_logger().info(
            f'drive_square started: {self.laps} lap(s), '
            f'side {self.side_length:.2f} m, waiting for first odometry'
        )

    def odom_callback(self, msg):
        # Odometry.pose is a PoseWithCovariance, whose .pose is the actual Pose.
        position = msg.pose.pose.position
        pose = Pose2D(position.x, position.y,
                      yaw_from_quaternion(msg.pose.pose.orientation))

        if self.pose is None:
            self.home = (pose.x, pose.y)
            self.start_yaw = pose.yaw
            self.segment_start = (pose.x, pose.y)
            self.get_logger().info(
                f'first odometry: x={pose.x:.3f} y={pose.y:.3f} yaw={pose.yaw:+.3f} '
                f'(frame {msg.header.frame_id!r} -> {msg.child_frame_id!r})'
            )
        self.pose = pose

    def tick(self):
        if self.pose is None:
            return  

        msg = Twist()

        if self.state == DRIVING:
            travelled = math.dist((self.pose.x, self.pose.y), self.segment_start)
            error = self.side_length - travelled
            if error <= self.distance_tolerance:
                self.get_logger().info(
                    f'side {self.sides_done + 1}: travelled {travelled:.3f} m '
                    f'(target {self.side_length:.3f}, error {error:+.3f})'
                )
                self.state = TURNING
                self.target_yaw = normalize_angle(
                    self.start_yaw + (self.sides_done + 1) * (math.pi / 2.0))
            else:
                msg.linear.x = clamp(
                    self.linear_gain * error,
                    self.min_linear_speed,
                    self.linear_speed)

        elif self.state == TURNING:
            error = normalize_angle(self.target_yaw - self.pose.yaw)
            if abs(error) <= self.angle_tolerance:
                self.sides_done += 1
                self.get_logger().info(
                    f'corner {self.sides_done}: yaw {self.pose.yaw:+.3f} '
                    f'(target {self.target_yaw:+.3f}, error {error:+.3f})'
                )
                if self.sides_done >= 4 * self.laps:
                    self.finish()
                    return
                self.state = DRIVING
                self.segment_start = (self.pose.x, self.pose.y)
            else:
                speed = clamp(
                    self.angular_gain * abs(error),
                    self.min_angular_speed,
                    self.angular_speed)
                msg.angular.z = math.copysign(speed, error)

        self.publisher.publish(msg)

    def finish(self):
        self.publisher.publish(Twist())  # explicit stop 
        self.timer.cancel()
        drift = math.dist((self.pose.x, self.pose.y), self.home)
        heading_error = normalize_angle(self.pose.yaw - self.start_yaw)
        self.get_logger().info(
            f'square complete: {drift:.3f} m from start, '
            f'heading off by {heading_error:+.3f} rad'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DriveSquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publisher.publish(Twist())  # do not leave the robot driving
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
