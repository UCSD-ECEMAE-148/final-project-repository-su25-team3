#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32, Bool


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop_node')

        # Parameters
        self.declare_parameter('stop_distance', 0.6)       # m
        self.declare_parameter('release_distance', 0.7)    # m
        self.declare_parameter('fov', math.radians(40.0))  # rad, total window (±20°)
        self.declare_parameter('center_angle', 0.0)        # rad, forward = 0
        self.declare_parameter('cmd_in_topic', '/cmd_vel_lane')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_safe')
        self.declare_parameter('scan_topic', '/scan')

        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.release_distance = float(self.get_parameter('release_distance').value)
        self.fov = float(self.get_parameter('fov').value)
        self.center_angle = float(self.get_parameter('center_angle').value)
        cmd_in_topic = self.get_parameter('cmd_in_topic').value
        cmd_out_topic = self.get_parameter('cmd_out_topic').value
        scan_topic = self.get_parameter('scan_topic').value

        self.obstructed = False
        self.latest_cmd = Twist()
        self.min_distance = float('inf')

        # Subscribers / Publishers
        self.cmd_sub = self.create_subscription(Twist, cmd_in_topic, self.cmd_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_cb, qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Twist, cmd_out_topic, 10)
        self.dist_pub = self.create_publisher(Float32, '/obstacle_distance', 10)
        self.block_pub = self.create_publisher(Bool, '/obstruction_active', 10)

        # log every 0.5s so logs aren't spammy
        self.log_timer = self.create_timer(0.5, self.log_tick)

        # 50 Hz republish loop
        self.timer = self.create_timer(0.02, self.tick)

        self.get_logger().info(
            f"SafetyStop watching {scan_topic}, gating {cmd_in_topic} -> {cmd_out_topic} "
            f"(stop<{self.stop_distance}m, release>{self.release_distance}m, "
            f"fov={math.degrees(self.fov):.1f}°)"
        )

    def cmd_cb(self, msg: Twist):
        self.latest_cmd = msg

    def scan_cb(self, scan: LaserScan):
        # Compute indices covering [center_angle - fov/2, center_angle + fov/2]
        ang_min = scan.angle_min
        ang_inc = scan.angle_increment
        n = len(scan.ranges)
        fov_min = self.center_angle - self.fov / 2.0
        fov_max = self.center_angle + self.fov / 2.0

        # Clamp to scan bounds
        i_min = max(0, int((fov_min - ang_min) / ang_inc)) if ang_inc != 0.0 else 0
        i_max = min(n - 1, int((fov_max - ang_min) / ang_inc)) if ang_inc != 0.0 else n - 1
        if i_min > i_max:
            i_min, i_max = i_max, i_min

        window = scan.ranges[i_min:i_max + 1]
        # Ignore 0.0 and inf
        finite = [r for r in window if r and math.isfinite(r)]
        min_r = min(finite) if finite else float('inf')

        self.min_distance = float(min_r)
        self.dist_pub.publish(Float32(data=self.min_distance))
        self.block_pub.publish(Bool(data=self.obstructed))

        # Hysteresis
        if self.obstructed:
            if min_r > self.release_distance:
                self.obstructed = False
        else:
            if min_r < self.stop_distance:
                self.obstructed = True

    def tick(self):
        out = Twist()
        out.angular.z = self.latest_cmd.angular.z
        out.linear.x = 0.0 if self.obstructed else self.latest_cmd.linear.x
        self.pub.publish(out)

    def log_tick(self):
        # Print current distance and state at 2 Hz
        d = self.min_distance
        d_str = "inf" if math.isinf(d) else f"{d:.2f} m"
        state = "BLOCKING" if self.obstructed else "CLEAR"
        self.get_logger().info(f"Obstacle distance: {d_str} | Gate: {state}")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
