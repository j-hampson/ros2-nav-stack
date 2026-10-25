"""Drive a list of waypoints by asking Nav2 to navigate to each in turn."""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from robot_lab.geometry import quaternion_from_yaw

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class Patrol(Node):

    def __init__(self):
        super().__init__('patrol')

        self.declare_parameter('waypoints', [1.5, 0.0, 3.0, 0.0])
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('loop', False)
        self.declare_parameter('face_travel', True)
        self.declare_parameter('server_timeout', 20.0)

        flat = list(self.get_parameter('waypoints').value)
        if len(flat) < 2 or len(flat) % 2:
            raise ValueError(
                f'waypoints needs an even count of at least 2 values, got {len(flat)}')
        self.waypoints = list(zip(flat[0::2], flat[1::2]))
        self.frame_id = self.get_parameter('frame_id').value
        self.loop = self.get_parameter('loop').value
        self.face_travel = self.get_parameter('face_travel').value
        self.server_timeout = self.get_parameter('server_timeout').value

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.index = 0
        self.lap = 1
        self.goal_handle = None
        self.last_feedback_log = -1.0
        self.results = []          # (waypoint, status text) per attempt

        self.get_logger().info(
            f'patrol: {len(self.waypoints)} waypoint(s) in frame {self.frame_id!r}'
            + (' looping' if self.loop else ''))

    def start(self):
        """Block until the action server exists, then send the first goal.        """
        self.get_logger().info('waiting for the navigate_to_pose action server...')
        if not self.client.wait_for_server(timeout_sec=self.server_timeout):
            self.get_logger().error(
                'no navigate_to_pose server. Is Nav2 running and activated?')
            return False
        self.get_logger().info('action server found')
        self.send_next()
        return True

    def goal_pose(self, target):
        """Build the goal, optionally facing the direction of travel."""
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = target

        yaw = 0.0
        if self.face_travel:
            # Aim along the leg being driven. For the first waypoint of a lap
            # there is no previous leg, so keep the default heading.
            if self.index > 0:
                previous = self.waypoints[self.index - 1]
            elif self.loop and self.lap > 1:
                previous = self.waypoints[-1]
            else:
                previous = None
            if previous is not None:
                yaw = math.atan2(target[1] - previous[1], target[0] - previous[0])

        q = quaternion_from_yaw(yaw)
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = q
        return pose

    def send_next(self):
        if self.index >= len(self.waypoints):
            if self.loop:
                self.index = 0
                self.lap += 1
                self.get_logger().info(f'--- starting lap {self.lap} ---')
            else:
                self.finish()
                return

        target = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose = self.goal_pose(target)

        self.get_logger().info(
            f'waypoint {self.index + 1}/{len(self.waypoints)}: '
            f'({target[0]:+.2f}, {target[1]:+.2f})')
        self.last_feedback_log = -1.0

        # send_goal_async returns a future for the goal HANDLE, not the result:
        # acceptance and completion are two separate round trips.
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        """The server has decided whether to accept the goal."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            # Rejection is not failure-to-reach: the server declined outright,
            # usually because it is inactive or the goal is malformed.
            self.get_logger().warn('  goal REJECTED by the server')
            self.record('REJECTED')
            self.advance()
            return
        self.goal_handle.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, msg):
        """Progress updates, roughly once a second while driving."""
        remaining = msg.feedback.distance_remaining
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_feedback_log >= 1.0:
            self.last_feedback_log = now
            self.get_logger().info(f'  {remaining:5.2f} m remaining')

    def on_result(self, future):
        status = future.result().status
        name = STATUS_NAMES.get(status, f'status {status}')
        self.get_logger().info(f'  -> {name}')
        self.record(name)
        self.advance()

    def record(self, outcome):
        self.results.append((self.waypoints[self.index], outcome))

    def advance(self):
        # Carry on after a failure rather than stopping
        self.index += 1
        self.goal_handle = None
        self.send_next()

    def finish(self):
        ok = sum(1 for _, s in self.results if s == 'SUCCEEDED')
        self.get_logger().info(f'patrol complete: {ok}/{len(self.results)} reached')
        for (x, y), outcome in self.results:
            self.get_logger().info(f'  ({x:+.2f}, {y:+.2f})  {outcome}')

    def cancel_active_goal(self):
        """Ask Nav2 to stop. Without this, Ctrl-C leaves the robot driving."""
        if self.goal_handle is not None:
            self.get_logger().info('cancelling active goal')
            self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = Patrol()
    try:
        if node.start():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.cancel_active_goal()
        rclpy.spin_once(node, timeout_sec=2.0)   # let the cancel reach the server
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
