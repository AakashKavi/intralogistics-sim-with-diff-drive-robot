#!/usr/bin/env python3
"""Safety layer between teleop and the controller: passes teleop commands
through unchanged, except forward motion is clamped to zero when something
is within stop_distance ahead (turning/reversing still pass through freely,
so you can steer out of it yourself). If forward stays blocked continuously
for wait_time, it briefly takes over to turn toward the more open side, then
hands control back to teleop."""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class TeleopSafetyFilter(Node):

    def __init__(self):
        super().__init__('teleop_safety_filter')

        self.declare_parameter('turn_speed', 0.4)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('forward_half_angle_deg', 20.0)
        self.declare_parameter('side_start_angle_deg', 25.0)
        self.declare_parameter('side_end_angle_deg', 90.0)
        self.declare_parameter('wait_time', 5.0)
        self.declare_parameter('turn_angle_deg', 80.0)

        self.turn_speed = self.get_parameter('turn_speed').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.forward_half_angle = math.radians(self.get_parameter('forward_half_angle_deg').value)
        self.side_start_angle = math.radians(self.get_parameter('side_start_angle_deg').value)
        self.side_end_angle = math.radians(self.get_parameter('side_end_angle_deg').value)
        self.wait_time = self.get_parameter('wait_time').value
        turn_angle = math.radians(self.get_parameter('turn_angle_deg').value)
        self.turn_duration = turn_angle / self.turn_speed

        self.cmd_pub = self.create_publisher(TwistStamped, '/diffbot_base_controller/cmd_vel', 10)
        self.forward_distance_pub = self.create_publisher(Float32, '/forward_obstacle_distance', 10)
        self.teleop_sub = self.create_subscription(
            TwistStamped, '/cmd_vel_teleop', self.teleop_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.latest_teleop_cmd = None
        self.latest_scan = None
        self.state = 'TELEOP'
        self.blocked_since = None
        self.turn_end_time = None
        self.turn_direction = 1

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            'teleop_safety_filter started: relaying /cmd_vel_teleop -> /diffbot_base_controller/cmd_vel')

    def teleop_callback(self, msg):
        self.latest_teleop_cmd = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

    @staticmethod
    def _index_for_angle(scan, angle):
        index = round((angle - scan.angle_min) / scan.angle_increment)
        return max(0, min(len(scan.ranges) - 1, index))

    def _min_range_in_cone(self, scan, angle_lo, angle_hi):
        lo = self._index_for_angle(scan, angle_lo)
        hi = self._index_for_angle(scan, angle_hi)
        values = [r for r in scan.ranges[lo:hi + 1] if scan.range_min <= r <= scan.range_max]
        return min(values) if values else float('inf')

    def _avg_range_in_cone(self, scan, angle_lo, angle_hi):
        lo = self._index_for_angle(scan, angle_lo)
        hi = self._index_for_angle(scan, angle_hi)
        values = [
            r if scan.range_min <= r <= scan.range_max else scan.range_max
            for r in scan.ranges[lo:hi + 1]
        ]
        return sum(values) / len(values) if values else 0.0

    def publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def control_loop(self):
        if self.latest_scan is None:
            return

        now = self.get_clock().now()
        scan = self.latest_scan

        min_ahead = self._min_range_in_cone(scan, -self.forward_half_angle, self.forward_half_angle)
        publishable_distance = min_ahead if math.isfinite(min_ahead) else scan.range_max
        self.forward_distance_pub.publish(Float32(data=publishable_distance))

        if self.state == 'TELEOP':
            cmd = self.latest_teleop_cmd
            linear_x = cmd.twist.linear.x if cmd is not None else 0.0
            angular_z = cmd.twist.angular.z if cmd is not None else 0.0

            blocked = linear_x > 0.0 and min_ahead < self.stop_distance

            if blocked:
                linear_x = 0.0
                if self.blocked_since is None:
                    self.blocked_since = now
                    self.get_logger().info(
                        f'Forward blocked ({min_ahead:.2f} m ahead), holding for {self.wait_time:.1f}s')
                elapsed = (now - self.blocked_since).nanoseconds / 1e9
                if elapsed >= self.wait_time:
                    left_avg = self._avg_range_in_cone(
                        scan, self.side_start_angle, self.side_end_angle)
                    right_avg = self._avg_range_in_cone(
                        scan, -self.side_end_angle, -self.side_start_angle)
                    self.turn_direction = 1 if left_avg >= right_avg else -1
                    self.turn_end_time = now + Duration(seconds=self.turn_duration)
                    self.state = 'TURNING'
                    side = 'left' if self.turn_direction == 1 else 'right'
                    self.get_logger().info(
                        f'Still blocked: left={left_avg:.2f}m right={right_avg:.2f}m -> auto-turning {side}')
            else:
                self.blocked_since = None

            self.publish_cmd(linear_x, angular_z)

        elif self.state == 'TURNING':
            self.publish_cmd(0.0, self.turn_direction * self.turn_speed)
            if now >= self.turn_end_time:
                self.state = 'TELEOP'
                self.blocked_since = None
                self.get_logger().info('Auto-turn complete, teleop back in control')


def main():
    rclpy.init()
    node = TeleopSafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
